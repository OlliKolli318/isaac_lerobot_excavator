"""ROS-independent sampling and action indexing on the Isaac timeline."""

from collections import OrderedDict
from dataclasses import dataclass
import math
import threading
import time

NANOSECONDS = 1_000_000_000


class ClockReset(RuntimeError):
    """A new simulation run invalidated the active episode/action chunk."""


class MissingSample(RuntimeError):
    """The required synchronized sample did not arrive."""


def stamp_ns(stamp):
    return int(stamp.sec) * NANOSECONDS + int(stamp.nanosec)


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    clock_ns: int
    run_id: str
    state: tuple
    action: tuple
    images: dict
    image_stamps: dict
    dagger: tuple = ()


class SimTimeline:
    def __init__(self, capacity=64, tolerance_s=0.002, grace_s=0.5, clock_capacity=None):
        clock_capacity = capacity if clock_capacity is None else clock_capacity
        if capacity < 2 or clock_capacity < 2 or not 0 <= tolerance_s < grace_s:
            raise ValueError("Synchronization requires capacities >= 2 and 0 <= tolerance < grace.")
        self.condition = threading.Condition()
        self.now_ns = None
        self.epoch = 0
        self.run_id = None
        self.capacity = capacity
        self.clock_capacity = clock_capacity
        self.tolerance_ns = round(tolerance_s * NANOSECONDS)
        self.grace_ns = round(grace_s * NANOSECONDS)
        self.streams = {k: OrderedDict() for k in ("sample", "camera2", "camera3", "clock")}
        self._retired_runs = set()
        self._run_arrived_before_reset = False

    def _clear(self):
        for stream in self.streams.values():
            stream.clear()

    def update_clock(self, ns):
        with self.condition:
            if self.now_ns is not None and ns < self.now_ns:
                self.epoch += 1
                if self.run_id and not self._run_arrived_before_reset:
                    self._retired_runs.add(self.run_id)
                self.run_id = None
                self._run_arrived_before_reset = False
                self._clear()
            self.now_ns = ns
            self.streams["clock"][ns] = None
            while len(self.streams["clock"]) > self.clock_capacity:
                self.streams["clock"].popitem(last=False)
            self.condition.notify_all()

    def put(self, key, ns, value, run_id=None):
        with self.condition:
            if key == "sample":
                if not run_id or run_id in self._retired_runs:
                    return
                if self.run_id is not None and run_id != self.run_id:
                    self._retired_runs.add(self.run_id)
                    self.epoch += 1
                    self._run_arrived_before_reset = self.now_ns is not None and ns < self.now_ns
                    self._clear()
                self.run_id = run_id
            stream = self.streams[key]
            stream[ns] = value
            while len(stream) > self.capacity:
                stream.popitem(last=False)
            self.condition.notify_all()

    def time(self):
        with self.condition:
            if self.now_ns is None:
                raise MissingSample("No Isaac /clock received. Start the simulation first.")
            return self.epoch, self.now_ns

    def wait_ready(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        with self.condition:
            while self.now_ns is None or self.run_id is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise MissingSample("Waiting for /clock and /kaivuri/sim_sample. Open and play the simulation-branch USD.")
                self.condition.wait(min(0.05, remaining))

    def _nearest(self, key, ns):
        stamps = [s for s in self.streams[key] if abs(s - ns) <= self.tolerance_ns]
        return min(stamps, key=lambda s: abs(s - ns)) if stamps else None

    def _missing_details(self, key, target_ns):
        """Called with the condition held; report actual retained timestamps."""
        stream = self.streams[key]
        if not stream:
            return {"buffer": "empty", "count": 0}
        first, last = min(stream), max(stream)
        nearest = min(stream, key=lambda stamp: abs(stamp - target_ns))
        position = ("before retained history" if target_ns < first else
                    "after latest received" if target_ns > last else "gap within retained history")
        return {"buffer": position, "count": len(stream), "first_ns": first,
                "last_ns": last, "nearest_ns": nearest, "offset_ns": nearest - target_ns}

    def wait_sample(self, target_ns, epoch, stopped=lambda: False):
        with self.condition:
            while not stopped():
                if self.epoch != epoch:
                    raise ClockReset("Isaac clock/run reset.")
                if self.now_ns is not None and self.now_ns >= target_ns:

                    stamps = {k: self._nearest(k, target_ns) for k in self.streams}

                    if all(s is not None for s in stamps.values()):
                        payload = self.streams["sample"][stamps["sample"]]
                        state, action, run_id = payload[:3]
                        return Sample(
                            stamps["sample"], stamps["clock"], run_id, state, action,
                            {k: self.streams[k][stamps[k]] for k in ("camera2", "camera3")},
                            {k: stamps[k] for k in ("camera2", "camera3")},
                            payload[3] if len(payload) > 3 else (),
                        )
                    if self.now_ns - target_ns > self.grace_ns:
                        missing = {key: self._missing_details(key, target_ns)
                                   for key, stamp in stamps.items() if stamp is None}
                        hints = []
                        if "clock" in missing:
                            hints.append("Check /clock delivery, publisher count and clock history.")
                        if "sample" in missing:
                            hints.append("Check synchronized state/action delivery.")
                        if "camera2" in missing or "camera3" in missing:
                            hints.append("Check camera timestamps and publication rate.")
                        raise MissingSample(
                            f"Missing synchronized sample at {target_ns / NANOSECONDS:.6f}s: {stamps}. "
                            f"Latest /clock={self.now_ns}ns; missing source details={missing}. "
                            + " ".join(hints)
                        )
                # Host time only makes waits interruptible: a paused /clock
                # never creates a frame or advances a policy action.
                self.condition.wait(0.05)
        return None


class SimRate:
    """Schedule frames at ``fps`` using simulation time in nanoseconds.

    Frame 0 is at ``origin_ns``. Each later frame has a target time computed
    from that origin and its index, avoiding accumulated rounding drift.
    These targets form a regular recording schedule even if /clock messages
    arrive at different times or at a different rate.

    ``after(ns)`` finds the first scheduled frame strictly later than ``ns``.
    ``next()`` returns the target time at ``self.index``, then advances the
    index so the following call returns the next frame's target time.
    """
    def __init__(self, fps, origin_ns=0):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Simulation FPS must be positive and finite.")
        self.fps, self.origin_ns, self.index = fps, origin_ns, 0

    def at(self, index):
        return self.origin_ns + round(index * NANOSECONDS / self.fps)
    # This returns the index of the next schedulable frame after the given timestamp, which is useful for
    # scheduling inference and action selection in the simulation loop.
    def after(self, ns):
        index = max(0, math.floor((ns - self.origin_ns) * self.fps / NANOSECONDS) + 1)
        while self.at(index) <= ns:
            index += 1
        return index

    def next(self):
        result = self.at(self.index)
        self.index += 1
        return result


class TimedChunk:
    """Actions retain their timestamps across pauses and inference delays."""
    def __init__(self, fps, origin_ns, epoch, original, processed):
        self.fps, self.origin_ns, self.epoch = fps, origin_ns, epoch
        self.original, self.processed = original, processed

    def index(self, ns):
        return round((ns - self.origin_ns) * self.fps / NANOSECONDS)

    def at(self, ns, epoch):
        i = self.index(ns)
        if epoch != self.epoch or ns < self.origin_ns or not 0 <= i < len(self.processed):
            return None
        return self.processed[i]

    def remaining(self, ns, epoch):
        if epoch != self.epoch:
            return 0
        return max(0, len(self.processed) - max(0, self.index(ns)))
