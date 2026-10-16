"""SmolVLA inference whose chunks and RTC latency use simulation time."""
from collections import deque
from contextlib import nullcontext
import logging
import math
import threading
import time

from numpy import rint

import torch
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.utils.feature_utils import build_dataset_frame
from .simulation_time import TimedChunk
from .inference_timing import InferenceTiming

logger = logging.getLogger(__name__)


class SimulationPolicy:
    def __init__(self, ctx):
        self.ctx = ctx
        self.cfg = ctx.runtime.cfg
        self.timeline = ctx.hardware.robot_wrapper.inner.timeline
        self.rtc = self.cfg.inference.type == "rtc"
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.chunk = None
        self.observation = None
        self.error = None
        self.processed_epoch = None
        self.generation = 0
        self.latencies = deque(maxlen=10)
        self.timing = InferenceTiming() if getattr(self.cfg, "measure_inference_time", False) else None
        if ctx.policy.policy.config.type != "smolvla":
            raise ValueError("Simulation rollout currently supports SmolVLA.")
        if any(type(s).__name__ == "RelativeActionsProcessorStep" and s.enabled
               for s in ctx.policy.preprocessor.steps):
            raise ValueError("Simulation rollout requires the excavator absolute joystick action contract.")

    def start(self, background=False):
        if self.rtc or background:
            self.thread = threading.Thread(target=self._run, name="SimulationRTC", daemon=True)
            self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                logger.warning("Inference still finishing; late results will be discarded.")
        if self.timing:
            # A timed-out worker may still finish another chunk after this snapshot.
            self.timing.report()

    def notify(self, obs, target_ns, epoch):
        with self.lock:
            self.observation = (obs, target_ns, epoch, self.generation)

    def invalidate(self):
        """Discard queued/in-flight actions across expert handovers."""
        with self.lock:
            self.generation += 1
            self.observation = None
            self.chunk = None

    def action_at(self, ns, epoch):
        with self.lock:
            if self.error:
                raise RuntimeError("Simulation policy inference failed") from self.error
            action = None if self.chunk is None else self.chunk.at(ns, epoch)
            return None if action is None else action.clone()

    def _run(self):
        last = None
        try:
            while not self.stop_event.wait(0.005):
                with self.lock:
                    observation, chunk = self.observation, self.chunk
                if observation is None or observation[1:] == last:
                    continue
                epoch, now = self.timeline.time()
                if observation[2] != epoch:
                    continue
                if chunk and chunk.remaining(now, epoch) > getattr(self.cfg.inference, 'queue_threshold', 1):
                    continue
                last = observation[1:]
                self.predict(observation)
        except Exception as exc:
            logger.exception("Simulation inference failed")
            with self.lock:
                self.error = exc

    def predict(self, observation=None):
        if observation is None:
            with self.lock:
                observation = self.observation
        if observation is None:
            return
        obs, origin, epoch = observation[:3]
        with self.lock:
            # Keep the original direct-predict API for base rollout/tests;
            # background observations always carry their handover generation.
            generation = observation[3] if len(observation) > 3 else self.generation
        policy = self.ctx.policy
        if (epoch, generation) != self.processed_epoch:
            # The inference owner resets processors. Never race a forward
            # pass with a processor reset in the control thread.
            policy.policy.reset()
            policy.preprocessor.reset()
            policy.postprocessor.reset()
            self.latencies.clear()
            self.processed_epoch = (epoch, generation)
            with self.lock:
                self.chunk = None
        with self.lock:
            chunk = self.chunk
        previous = None
        if self.rtc and chunk and chunk.epoch == epoch:
            previous = chunk.original[max(0, chunk.index(origin)):].clone()
            if not len(previous):
                previous = None
        device = torch.device(self.cfg.device or policy.policy.config.device)
        if self.timing:
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            pipeline_start = time.perf_counter()
        batch = build_dataset_frame(self.ctx.data.hw_features, obs, prefix="observation")
        batch = prepare_observation_for_inference(
            batch, device, self.cfg.task, self.ctx.hardware.robot_wrapper.robot_type)
        batch["task"] = [self.cfg.task]
        amp = (torch.autocast(device_type="cuda") if device.type == "cuda"
               and policy.policy.config.use_amp else nullcontext())
        # RTC locally enables autograd for guidance, so avoid inference_mode.
        with torch.no_grad(), amp:
            inputs = policy.preprocessor(batch)
            kwargs = {}
            if self.rtc:
                # One additional control slot accounts for sending commands
                # ahead for their next defined simulation tick.
                delay = math.ceil(max(self.latencies, default=0) * self.cfg.fps / 1e9) + 1
                kwargs = {"inference_delay": delay, "prev_chunk_left_over": previous}
            if self.timing:
                if device.type == "cuda":
                    torch.cuda.synchronize(device)
                model_start = time.perf_counter()
            actions = policy.policy.predict_action_chunk(inputs, **kwargs)
            if self.timing:
                if device.type == "cuda":
                    torch.cuda.synchronize(device)
                model_ms = (time.perf_counter() - model_start) * 1000
            original = actions.squeeze(0).detach().clone()
            processed = policy.postprocessor(actions).squeeze(0).detach().cpu()
        if self.timing:
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            pipeline_ms = (time.perf_counter() - pipeline_start) * 1000
        if not self.rtc:
            count = policy.policy.config.n_action_steps
            original, processed = original[:count], processed[:count]
        current_epoch, now = self.timeline.time()
        if self.timing:
            # Include completed computation even if a reset/handover discards its actions.
            self.timing.record(model_ms, pipeline_ms)
        if self.stop_event.is_set() or current_epoch != epoch:
            return
        self.latencies.append(max(0, now - origin))
        chunk = TimedChunk(self.cfg.fps, origin, epoch, original, processed)
        with self.lock:
            if generation != self.generation or self.stop_event.is_set():
                return
            self.chunk = chunk
        if chunk.remaining(now, epoch) <= 1:
            logger.warning("The chunk expired during inference. Increase the usable chunk length or reduce inference latency.")
        logger.debug("Chunk ready: simulation latency %.3fs", (now - origin) / 1e9)
