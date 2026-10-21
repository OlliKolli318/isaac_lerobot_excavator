"""Intervention DAgger using atomic Isaac samples, not a wall-time rollout loop.

The expert is the in-scene Xbox. The teleop subscriber is deliberately not used
as an expert source: /kaivuri/action contains the *selected* action, including
policy actions. JointState.effort is protocol metadata, not physical effort.
"""
from contextlib import ExitStack
import copy
import logging
import queue
import secrets
import time

import numpy as np
from lerobot.rollout import build_rollout_context
from lerobot.rollout.configs import BaseStrategyConfig
from lerobot.rollout.strategies.dagger import DAggerEvents
from lerobot.utils.keyboard_input import create_key_listener
from lerobot.utils.feature_utils import build_dataset_frame
from .kaivuri_record import _make_dataset
from .ros_bridge import RosNodeContext, make_recording_mode_qos
from .simulation_policy import SimulationPolicy
from .simulation_record import TIME_FEATURES
from .simulation_time import MissingSample, SimRate

PHASES = {0: 'idle', 1: 'autonomous', 2: 'paused', 3: 'correcting', 4: 'off', 5: 'armed'}


def sample_control(sample):
    tag = sample.dagger
    if (len(tag) != 8 or tag[0] != 2 or any(not np.isfinite(x) for x in tag)
            or any(int(x) != x for x in tag) or tag[3] not in PHASES
            or tag[4] not in (0, 1) or tag[5] not in range(5)
            or not 0 <= tag[1] < 2**52 or not 0 <= tag[2] < 2**31
            or any(not 0 <= x < 2**52 for x in tag[6:])):
        raise ValueError('Scene lacks simulation DAgger v2 metadata. Run configure_splat_dagger.py and reopen env.usd.')
    return f'{int(tag[1])}:{int(tag[2])}:{PHASES[int(tag[3])]}', PHASES[int(tag[5])], bool(tag[4])


def init_simulation_dagger_keyboard(events, link, cfg):
    def dispatch(name):
        if name == 'esc':
            events.stop_recording.set()
        elif name == 'tab':
            # Never use Tab to start recording, including legacy CLI defaults.
            return
        elif name == cfg.pause_resume:
            link.commands.put('toggle_policy')
        elif name == cfg.upload:
            events.upload_requested.set()
    return create_key_listener(dispatch, controls_help=(
        f'{cfg.pause_resume}=policy pause/resume (Xbox LT); '
        'Xbox RT=pause/save; sticks/A=automatic correction start; ESC=exit; Tab unused'))


class DaggerLink:
    """A steady-time safety lease only; never a recording/inference clock."""
    def __init__(self, continue_after_success=False):
        from std_msgs.msg import String
        from rclpy.clock import Clock, ClockType
        self.String = String
        self.ros = RosNodeContext('kaivuri_simulation_dagger')
        with ExitStack() as cleanup:
            cleanup.callback(self.ros.disconnect)
            self.ros.connect()
            self.commands: queue.SimpleQueue[str] = queue.SimpleQueue()
            self.session, self.generation = secrets.randbelow(2**52 - 1) + 1, 0
            self.request = f'{self.session}:0:idle'
            self.ack = ''
            self.ack_at = -float('inf')
            self.request_at = time.monotonic()
            node = self.ros.node
            self.publisher = node.create_publisher(String, '/kaivuri/dagger/request', 10)
            self.continue_after_success = continue_after_success
            self.continue_publisher = node.create_publisher(String, '/kaivuri/dagger/continue_after_success', 10)
            self.status = node.create_publisher(String, '/kaivuri/episode/status', 10)
            self.command_sub = node.create_subscription(String, '/kaivuri/episode_command',
                                                       lambda m: self.commands.put(m.data.strip()), 10)
            self.ack_sub = node.create_subscription(String, '/kaivuri/dagger/state', self._ack,
                                                   make_recording_mode_qos())
            self.timer = node.create_timer(.2, self.heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.heartbeat()
            cleanup.pop_all()


    def _ack(self, msg):
        self.ack, self.ack_at = msg.data, time.monotonic()

    def heartbeat(self):
        self.publisher.publish(self.String(data=self.request))
        # Bind continuation to this exact autonomous session/generation. A
        # stale message cannot enable it after a pause, handover or lease expiry.
        self.continue_publisher.publish(self.String(
            data=self.request if self.continue_after_success and self.request.endswith(':autonomous') else ''))

    def phase(self, phase):
        self.generation += 1
        self.request = f'{self.session}:{self.generation}:{phase}'
        self.request_at = time.monotonic()
        self.heartbeat()
        logging.info('DAgger requested %s (generation %d)', phase, self.generation)

    def check(self):
        now = time.monotonic()
        if now - self.request_at > 5 and (self.ack != self.request or now - self.ack_at > 2):
            raise RuntimeError('DAgger bridge acknowledgement lost/rejected; control held. Restart the simulation bridge and collector.')

    def report(self, status):
        self.status.publish(self.String(data=status))

    def close(self):
        from rclpy.duration import Duration
        # Off releases ownership but the bridge remains disabled. If delivery
        # fails, lease expiry holds instead of reverting to autonomous motion.
        try:
            self.phase('off')
            self.ros.node.destroy_timer(self.timer)
            self.publisher.wait_for_all_acked(timeout=Duration(seconds=1))
        finally:
            self.ros.disconnect()


class SimulationDagger:
    def __init__(self, ctx, link, events):
        self.ctx, self.link, self.events = ctx, link, events
        self.cfg = ctx.runtime.cfg
        self.robot = ctx.hardware.robot_wrapper.inner
        self.dataset = ctx.data.dataset
        self.engine = SimulationPolicy(ctx)
        self.active = False
        self.finish = None
        self.ready = False
        self.saved = self.attempt_saved = 0
        self.previous_stamps = None
        self.last_target = None
        self.started_ns = None
        self.attempt_ticks = 0
        self.observed_phase = None
        self.next_phase = None
        self.trigger_counts = None

    def request(self, phase):
        self.engine.invalidate()
        self.link.phase(phase)

    def handover(self, command):
        if not self.active or self.finish is not None:
            return
        if command == 'pause_save':
            destination = 'armed'
        elif self.next_phase is not None:
            return  # Ignore another toggle while a handover is pending.
        else:
            destination = 'armed' if self.link.request.endswith(':autonomous') else 'autonomous'
        self.next_phase = destination
        # Acknowledge a hard hold before encoding. Do not re-arm expert input
        # or resume policy while the dataset writer is still busy saving.
        self.request('paused')

    def gamepad_commands(self, sample):
        counts = tuple(int(x) for x in sample.dagger[6:8])
        previous, self.trigger_counts = self.trigger_counts, counts
        if previous is None or counts == previous:
            return
        if any(a < b for a, b in zip(counts, previous, strict=True)):
            raise RuntimeError('DAgger gamepad counters reset; restart the collector.')
        # RT wins if both triggers were pressed between two recorder samples.
        self.handover('pause_save' if counts[1] > previous[1] else 'toggle_policy')

    def poll(self):
        self.link.check()
        while not self.link.commands.empty():
            command = self.link.commands.get()
            if command == 'start' and not self.active and self.ready:
                self.active = True
                self.next_phase = None
                self.attempt_saved = self.attempt_ticks = 0
                self.events.reset()
                self.request('autonomous')
            elif command in ('end_save', 'end_discard') and self.active and self.finish is None:
                self.finish = command
                self.next_phase = None
                self.request('idle')
            elif command in ('toggle_policy', 'pause_save'):
                self.handover(command)
        if self.events.upload_requested.is_set():
            self.events.upload_requested.clear()
            logging.info('Simulation DAgger uploads only at clean exit with --dataset.push_to_hub=true.')
        return self.events.stop_recording.is_set() or self.ctx.runtime.shutdown_event.is_set()

    def save_segment(self, discard=False):
        if self.dataset.has_pending_frames():
            if discard:
                self.dataset.clear_episode_buffer()
                logging.info('Discarded current unsaved DAgger segment.')
            else:
                self.dataset.save_episode()
                self.saved += 1
                self.attempt_saved += 1
                logging.info('Saved DAgger segment %d (dataset episodes: %d)', self.saved, self.dataset.num_episodes)
        self.previous_stamps = self.last_target = None

    def record(self, sample, target, obs, correcting):
        stamps = (sample.clock_ns, sample.stamp_ns, sample.image_stamps['camera2'], sample.image_stamps['camera3'])
        if self.previous_stamps and any(a <= b for a, b in zip(stamps, self.previous_stamps, strict=True)):
            raise MissingSample('A source frame would be reused in DAgger; refusing mislabeled data.')
        if self.last_target is not None and abs(target - self.last_target - 1e9 / self.cfg.fps) > 1.1:
            raise MissingSample('Noncontiguous DAgger recording grid; refusing to compress a missing tick.')
        action = self.robot.action_from_sample(sample)
        frame = {**build_dataset_frame(self.dataset.features, obs, prefix='observation'),
                 **build_dataset_frame(self.dataset.features, action, prefix='action'),
                 'intervention': np.array([correcting], dtype=bool), 'task': self.cfg.task}
        frame.update({k: np.array([ns], dtype=np.int64) for k, ns in zip(TIME_FEATURES, stamps, strict=True)})
        self.dataset.add_frame(frame)
        self.previous_stamps, self.last_target = stamps, target

    def infer(self, sample, target, epoch, rate, obs_raw, obs):
        self.engine.notify(obs, target, epoch)
        current_epoch, now = self.robot.timeline.time()
        if current_epoch != epoch:
            return
        due = rate.at(rate.after(max(now, target)))
        action = self.engine.action_at(due, epoch)
        command = dict(zip(self.robot.action_features, (0., 0., 0., 0., sample.action[-1]), strict=True))
        if action is not None:
            keys = self.ctx.data.ordered_action_keys
            if len(action) != len(keys):
                raise ValueError('Policy action dimension differs from the excavator contract.')
            command = self.ctx.processors.robot_action_processor(
                ({key: action[i].item() for i, key in enumerate(keys)}, obs_raw))
        self.robot.send_action_at_sim_time(command, due, sample.run_id + '#' + self.link.request)

    def run(self):
        self.engine.start(background=True)
        epoch, start = self.robot.timeline.time()
        rate = SimRate(self.cfg.fps)
        rate.index = rate.after(start)
        preflight_deadline = time.monotonic() + 15
        while not self.poll():
            target = rate.next()
            sample = self.robot.timeline.wait_sample(target, epoch, self.poll)
            if sample is None:
                break
            control, phase, enabled = sample_control(sample)
            self.gamepad_commands(sample)
            ours = control.startswith(str(self.link.session) + ':')
            if not self.ready:
                if control == self.link.request and phase == 'idle':
                    self.ready = True
                    self.started_ns = target
                    logging.info('Ready: LB=start; LT/Space=policy pause/resume; sticks/A=start correction; '
                                 'RT=pause and save segment; RB=end/save; X=end/discard current; Esc=exit. Tab unused.')
                elif time.monotonic() > preflight_deadline:
                    raise RuntimeError('Isaac has not acknowledged DAgger idle. Reopen patched env.usd and start Play.')
                continue
            if not ours:
                raise RuntimeError('Scene DAgger ownership changed; stopping without saving partial data.')
            if phase != self.observed_phase:
                logging.info('DAgger %s acknowledged at simulation %.6fs', phase, sample.stamp_ns / 1e9)
                self.observed_phase = phase
            if self.active and phase in ('autonomous', 'correcting'):
                self.attempt_ticks += 1
            recording = (self.active and enabled and
                         (phase == 'correcting' or (phase == 'autonomous' and self.cfg.strategy.record_autonomous)))
            if recording:
                raw = self.robot.observation_from_sample(sample)
                obs = self.ctx.processors.robot_observation_processor(raw)
                self.record(sample, target, obs, phase == 'correcting')
            else:
                # Encoding only while idle/paused, never while the expert is
                # driving. A disabled watchdog during policy execution aborts
                # an existing segment rather than silently skipping time.
                if self.dataset.has_pending_frames() and phase in ('autonomous', 'correcting'):
                    raise MissingSample('Control was disabled mid-segment; partial segment discarded.')
                if control == self.link.request and control.rsplit(':', 1)[-1] in ('idle', 'paused'):
                    # Wait for the hard-hold acknowledgement before potentially
                    # slow encoding. Then skip its interval before re-arming;
                    # otherwise camera history could expire during save_episode.
                    self.save_segment(discard=self.finish == 'end_discard')
            if self.next_phase is not None and control == self.link.request and phase == 'paused':
                destination, self.next_phase = self.next_phase, None
                self.save_segment()
                # Skip only the acknowledged hard-hold/encoding interval.
                # Armed-but-waiting ticks must be read so first intent is kept.
                _, now = self.robot.timeline.time()
                rate.index = max(rate.index, rate.after(now))
                self.request(destination)
            if self.finish and control == self.link.request and phase == 'idle':
                self.save_segment(discard=self.finish == 'end_discard')
                self.link.report('saved' if self.finish == 'end_save' and self.attempt_saved else 'discarded')
                self.active, self.finish = False, None
                logging.info('Attempt finished. LB starts another attempt.')
            if self.saved >= self.cfg.strategy.num_episodes:
                if not self.active:
                    break
                if self.finish is None:
                    self.finish = 'end_save'
                    self.next_phase = None
                    self.request('idle')
            if self.active and self.finish is None and self.attempt_ticks / self.cfg.fps >= self.cfg.dataset.episode_time_s:
                self.finish = 'end_save'
                self.next_phase = None
                self.request('idle')
            if self.cfg.duration > 0 and self.started_ns is not None and (target - self.started_ns) / 1e9 >= self.cfg.duration:
                break
            if self.active and self.finish is None and phase == 'autonomous' and control == self.link.request:
                raw = self.robot.observation_from_sample(sample)
                obs = self.ctx.processors.robot_observation_processor(raw)
                self.infer(sample, target, epoch, rate, raw, obs)
            if control == self.link.request and control.rsplit(':', 1)[-1] in ('idle', 'paused'):
                # No frames are recorded across a pause/encoding interval.
                _, now = self.robot.timeline.time()
                rate.index = max(rate.index, rate.after(now))


def _discard_and_finalize(dataset):
    try:
        if dataset.has_pending_frames():
            dataset.clear_episode_buffer()
    finally:
        dataset.finalize()


def run_dagger(cfg, stop):
    if getattr(cfg, 'continue_after_success', False) and cfg.strategy.record_autonomous:
        raise ValueError('continue_after_success requires strategy.record_autonomous=false to avoid recording log teleports.')
    if cfg.teleop.type != 'isaac_excavator_action':
        raise ValueError('Simulation DAgger uses --teleop.type=isaac_excavator_action (the in-scene Xbox).')
    if cfg.strategy.input_device != 'keyboard':
        raise ValueError('Simulation DAgger uses Xbox controls with keyboard fallback; keep strategy.input_device=keyboard.')
    if cfg.dataset.fps != cfg.fps or cfg.strategy.num_episodes <= 0 or cfg.dataset.episode_time_s <= 0:
        raise ValueError('Use matching fps/dataset.fps, positive num_episodes and episode_time_s.')
    if not cfg.dataset.repo_id.split('/')[-1].startswith('rollout_'):
        raise ValueError('DAgger dataset repo name must start with rollout_.')
    if cfg.strategy.target_video_file_size_mb is not None:
        raise ValueError('Simulation DAgger rotates on simulation episode duration/pauses, not video file size.')
    if not cfg.task.strip():
        raise ValueError('Set --dataset.single_task (or --task) to the training task instruction.')
    if cfg.robot.manage_control_mode or cfg.robot.start_mode != 'unchanged':
        raise ValueError('DAgger owns mode switching: use robot.manage_control_mode=false and robot.start_mode=unchanged.')
    # Load the training checkpoint processors/stats without replacing them
    # with statistics from the new, initially empty correction dataset.
    base = copy.copy(cfg)
    base.strategy, base.dataset, base.teleop = BaseStrategyConfig(), None, None
    ctx = build_rollout_context(base, stop)
    ctx.runtime.cfg = cfg
    robot = ctx.hardware.robot_wrapper.inner
    dataset = link = listener = runner = None
    clean_exit = False
    try:
        features = copy.deepcopy(ctx.data.dataset_features)
        if not cfg.dataset.video:
            for feature in features.values():
                if feature['dtype'] == 'video':
                    feature['dtype'] = 'image'
        features['intervention'] = {'dtype': 'bool', 'shape': (1,), 'names': None}
        for key in TIME_FEATURES:
            features[key] = {'dtype': 'int64', 'shape': (1,), 'names': None}
        dataset = _make_dataset(cfg, robot, features)
        ctx.data.dataset, ctx.data.dataset_features = dataset, features
        link = DaggerLink(continue_after_success=getattr(cfg, 'continue_after_success', False))
        events = DAggerEvents()
        listener = init_simulation_dagger_keyboard(events, link, cfg.strategy.keyboard)
        if listener is None:
            raise RuntimeError('No keyboard listener available for safe DAgger handover.')
        runner = SimulationDagger(ctx, link, events)
        runner.run()
        clean_exit = True
    finally:
        # Every resource must be released even when a hold request or encoder fails.
        with ExitStack() as cleanup:
            if robot.is_connected:
                cleanup.callback(robot.disconnect)
            if link:
                cleanup.callback(link.close)
            if dataset:
                cleanup.callback(_discard_and_finalize, dataset)
            if listener:
                cleanup.callback(listener.stop)
            if runner:
                cleanup.callback(runner.engine.stop)
            if link:
                link.phase('idle')
                if runner and runner.active:
                    link.report('discarded')
    if clean_exit and cfg.dataset.push_to_hub and dataset.num_episodes:
        dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
    return dataset
