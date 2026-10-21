"""Collect fixed-rate samples using source timestamps from Isaac."""
import numpy as np
from lerobot.utils.feature_utils import build_dataset_frame
from .simulation_time import MissingSample, SimRate

TIME_FEATURES = ("clock_time_ns", "sim_time_ns", "camera2_time_ns", "camera3_time_ns")


def simulation_record_loop(robot, events, fps, dataset, control_time_s, single_task,
                           robot_observation_processor, teleop_action_processor, **kwargs):
    epoch, start_ns = robot.timeline.time()
    rate = SimRate(fps)
    rate.index = rate.after(start_ns)
    first_ns = rate.at(rate.index)
    previous_stamps = None
    while not events["stop_recording"] and not events["exit_early"]:
        target_ns = rate.next()
        if (target_ns - first_ns) / 1e9 >= control_time_s:
            break
        sample = robot.timeline.wait_sample(
            target_ns, epoch, lambda: events["stop_recording"] or events["exit_early"])
        if sample is None:
            break
        obs = robot.observation_from_sample(sample)
        obs_processed = robot_observation_processor(obs)
        # Isaac already applies the gamepad action. Record the selected action
        # paired atomically with this state, instead of reading teleop later.
        action = teleop_action_processor((robot.action_from_sample(sample), obs))
        frame = {**build_dataset_frame(dataset.features, obs_processed, prefix="observation"),
                 **build_dataset_frame(dataset.features, action, prefix="action"),
                 "task": single_task}
        stamps = (sample.clock_ns, sample.stamp_ns, sample.image_stamps["camera2"], sample.image_stamps["camera3"])
        if previous_stamps is not None and any(a <= b for a, b in zip(stamps, previous_stamps, strict=True)):
            raise MissingSample("A source frame would be reused. Lower FPS or tighten sync_tolerance_s.")
        previous_stamps = stamps
        frame.update({k: np.array([ns], dtype=np.int64) for k, ns in zip(TIME_FEATURES, stamps, strict=True)})
        if "intervention" in dataset.features:
            # Manual demonstrations appended to a DAgger aggregate are not
            # policy interventions, matching dagger_aggregate's demo labels.
            frame["intervention"] = np.array([False], dtype=bool)
        # Installed LeRobot owns timestamp=frame_index/fps. Since every accepted
        # row occupies one simulation grid slot, that is elapsed simulation time.
        dataset.add_frame(frame)
        if kwargs.get("display_data"):
            from lerobot.utils.visualization_utils import log_visualization_data
            log_visualization_data(kwargs["display_mode"], observation=obs_processed, action=action,
                                   compress_images=kwargs["display_compressed_images"])
