"""Execute SmolVLA actions on Isaac's /clock grid."""
from contextlib import ExitStack
import logging
from dataclasses import dataclass

# Importing the recorder configures Windows DLL lookup before loading LeRobot.
from .kaivuri_record import LocalDatasetRecordConfig

from lerobot.configs import parser
from lerobot.rollout import RolloutConfig, build_rollout_context
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.process import ProcessSignalHandler
from lerobot.utils.utils import init_logging
from .simulation_policy import SimulationPolicy
from .simulation_time import ClockReset, SimRate


@dataclass
class SimulationRolloutConfig(RolloutConfig):
    dataset: LocalDatasetRecordConfig | None = None
    # Joystick commands cannot be interpreted as absolute joint positions.
    return_to_initial_position: bool = False
    measure_inference_time: bool = False
    continue_after_success: bool = False


def run_simulation_rollout(ctx):
    cfg, stop = ctx.runtime.cfg, ctx.runtime.shutdown_event
    robot = ctx.hardware.robot_wrapper.inner
    engine = None
    last_sent_ns, last_run_id, last_claw = -1, None, 1.0
    try:
        engine = SimulationPolicy(ctx)
        engine.start()
        epoch, start = robot.timeline.time()
        rate = SimRate(cfg.fps)
        rate.index = rate.after(start)
        logging.info("Control: %.1f actions per simulation second; duration %.1f simulated seconds.", cfg.fps, cfg.duration)
        while not stop.is_set():
            try:
                current_epoch, now = robot.timeline.time()
                if current_epoch != epoch:
                    raise ClockReset("Clock restarted")
                if cfg.duration > 0 and (now - start) / 1e9 >= cfg.duration:
                    break
                target = rate.next()
                sample = robot.timeline.wait_sample(target, epoch, stop.is_set)
                if sample is None:
                    break
                last_claw = sample.action[-1]
                obs = robot.observation_from_sample(sample)
                processed = ctx.processors.robot_observation_processor(obs)
                engine.notify(processed, target, epoch)
                current_epoch, now = robot.timeline.time()
                if current_epoch != epoch:
                    raise ClockReset("Clock restarted during observation processing")
                due_index = rate.after(max(now, target))
                due = rate.at(due_index)
                if not engine.rtc and engine.action_at(due, epoch) is None:
                    engine.predict()
                    current_epoch, now = robot.timeline.time()
                    if current_epoch != epoch:
                        raise ClockReset("Clock restarted during inference")
                    due_index = rate.after(now)
                    due = rate.at(due_index)
                action = engine.action_at(due, epoch)
                if action is None:
                    command = dict(zip(robot.action_features, (0., 0., 0., 0., last_claw), strict=True))
                else:
                    keys = ctx.data.ordered_action_keys
                    if len(action) != len(keys):
                        raise ValueError("Policy output dimension does not match excavator actions.")
                    action_dict = {key: action[i].item() for i, key in enumerate(keys)}
                    command = ctx.processors.robot_action_processor((action_dict, obs))
                    last_claw = command[list(robot.action_features)[-1]]
                robot.send_action_at_sim_time(command, due, sample.run_id)
                last_sent_ns, last_run_id = due, sample.run_id
                # Skip expired slots after a stall. No burst of old commands.
                rate.index = max(rate.index, due_index)
            except ClockReset:
                logging.warning("Isaac reset: invalidating chunks and restarting duration.")
                epoch, start = robot.timeline.time()
                rate.index = rate.after(start)
                last_sent_ns, last_run_id = -1, None
    finally:
        # Supersede a queued command before waiting for the inference thread.
        try:
            if robot.is_connected:
                _, now = robot.timeline.time()
                run_id = robot.timeline.run_id
                if run_id and run_id == last_run_id:
                    robot.send_action_at_sim_time(
                        dict(zip(robot.action_features, (0., 0., 0., 0., last_claw), strict=True)),
                        max(now + 1, last_sent_ns + 1), run_id)
        finally:
            with ExitStack() as cleanup:
                if ctx.hardware.teleop and ctx.hardware.teleop.is_connected:
                    cleanup.callback(ctx.hardware.teleop.disconnect)
                if robot.is_connected:
                    cleanup.callback(robot.disconnect)
                if engine:
                    cleanup.callback(engine.stop)


@parser.wrap()
def rollout(cfg: SimulationRolloutConfig):
    init_logging()
    if cfg.interpolation_multiplier != 1 or cfg.use_torch_compile or cfg.display_data:
        raise ValueError("Use interpolation_multiplier=1, use_torch_compile=false and display_data=false.")
    if cfg.return_to_initial_position:
        raise ValueError("return_to_initial_position must be false for joystick actions.")
    SimRate(cfg.fps)
    cfg.robot.simulation_time = True
    signal_handler = ProcessSignalHandler(use_threads=True, display_pid=False)
    if cfg.strategy.type == 'dagger':
        from .simulation_dagger import run_dagger
        return run_dagger(cfg, signal_handler.shutdown_event)
    if cfg.strategy.type != "base" or cfg.dataset is not None or cfg.teleop is not None:
        raise ValueError("Use strategy.type=base without dataset/teleop, or strategy.type=dagger with both.")
    ctx = build_rollout_context(cfg, signal_handler.shutdown_event)
    # The standard engine is constructed but never started. It owns wall-time
    # accounting, so the project engine below handles simulation scheduling.
    run_simulation_rollout(ctx)


def main():
    register_third_party_plugins()
    rollout()


if __name__ == "__main__":
    main()
