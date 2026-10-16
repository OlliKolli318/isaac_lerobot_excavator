"""LeRobot recorder with ROS 2 episode completion for the Isaac excavator.

This is intentionally project-owned.  It reuses LeRobot's robot, dataset,
processor, and record-loop implementations, while adding one small boundary:
Isaac can publish ``end_save`` on ``/kaivuri/episode_command`` to finish and
save the active episode.  After ``dataset.save_episode()`` completes this
recorder publishes ``saved`` on ``/kaivuri/episode/status``.  Isaac waits for
that acknowledgement before it relocates the task objects.
"""

import logging
import os
import sys
import threading
from dataclasses import asdict, dataclass
from pathlib import Path
from pprint import pformat


_DLL_DIRECTORY_HANDLES = []


def _configure_windows_dll_search_path() -> None:
    """Make a pip console script work inside the Windows Pixi ROS environment.

    ``pixi run`` adds ``Library\\bin`` to the DLL lookup path. A pip-generated
    ``.exe`` entry point does not necessarily inherit that setup, causing OpenCV
    to fail before LeRobot starts. Add the directory before importing LeRobot.
    """
    if sys.platform != "win32" or not hasattr(os, "add_dll_directory"):
        return

    executable_dir = Path(sys.executable).resolve().parent
    for environment_root in (executable_dir, executable_dir.parent):
        library_bin = environment_root / "Library" / "bin"
        if library_bin.is_dir():
            _DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(str(library_bin)))


_configure_windows_dll_search_path()

from lerobot.configs import parser
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.processor import RobotProcessorPipeline, make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.scripts import lerobot_record as upstream
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_visualization, shutdown_visualization
from .simulation_time import ClockReset, MissingSample
from .simulation_record import TIME_FEATURES, simulation_record_loop


@dataclass
class KaivuriRecordConfig(upstream.RecordConfig):
    """Standard LeRobot recorder settings plus the Isaac episode handshake."""

    episode_command_topic: str = "/kaivuri/episode_command"
    episode_status_topic: str = "/kaivuri/episode/status"
    start_command: str = "start"
    end_save_command: str = "end_save"
    end_discard_command: str = "end_discard"
    saved_status: str = "saved"
    discarded_status: str = "discarded"
    time_source: str = "simulation"


def _without_feature_info(value):
    """Remove codec details that LeRobot intentionally excludes on resume."""
    if isinstance(value, dict):
        return {key: _without_feature_info(item) for key, item in value.items() if key != "info"}
    if isinstance(value, list):
        return [_without_feature_info(item) for item in value]
    if isinstance(value, tuple):
        return [_without_feature_info(item) for item in value]
    return value


def _check_resume_compatibility(dataset, robot, fps: int, features: dict) -> None:
    """Validate append safety without LeRobot's optional deepdiff dependency."""
    from lerobot.utils.constants import DEFAULT_FEATURES

    expected_features = {**features, **DEFAULT_FEATURES}
    # Aggregation adds this label to demonstrations and DAgger corrections.
    # The simulation recorder can append ordinary demonstrations with False.
    # Keep wall-time recording strict: its upstream loop does not emit labels.
    if "intervention" in dataset.features and all(key in features for key in TIME_FEATURES):
        expected_features.setdefault("intervention", {"dtype": "bool", "shape": (1,), "names": None})
    mismatches = []
    if dataset.meta.robot_type != robot.robot_type:
        mismatches.append(f"robot_type: expected {robot.robot_type!r}, got {dataset.meta.robot_type!r}")
    if dataset.fps != fps:
        mismatches.append(f"fps: expected {fps!r}, got {dataset.fps!r}")
    actual = _without_feature_info(dataset.features)
    expected = _without_feature_info(expected_features)
    for key in sorted(actual.keys() | expected.keys()):
        if key not in actual:
            mismatches.append(f"features.{key}: missing from dataset")
        elif key not in expected:
            mismatches.append(f"features.{key}: unsupported extra dataset feature")
        elif actual[key] != expected[key]:
            mismatches.append(f"features.{key}: expected {expected[key]!r}, got {actual[key]!r}")
    if mismatches:
        raise ValueError("Dataset metadata compatibility check failed:\n" + "\n".join(mismatches))


class EpisodeCommandBridge:
    """Owns the recorder-side half of the ROS episode completion handshake."""

    def __init__(self, events: dict, cfg: KaivuriRecordConfig) -> None:
        self._events = events
        self._cfg = cfg
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._active_episode = False
        self._start_requested = False
        self._end_command = None
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._status_publisher = None
        self._started_rclpy = False

    def start(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from std_msgs.msg import String

        if not rclpy.ok():
            rclpy.init(args=None)
            self._started_rclpy = True

        self._node = rclpy.create_node("kaivuri_lerobot_native_recorder")
        self._status_publisher = self._node.create_publisher(String, self._cfg.episode_status_topic, 10)
        self._node.create_subscription(String, self._cfg.episode_command_topic, self._on_command, 10)
        self._executor = MultiThreadedExecutor(num_threads=1)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        logging.info(
            "Listening for start/save/discard commands on %s; publishing episode status on %s.",
            self._cfg.episode_command_topic,
            self._cfg.episode_status_topic,
        )

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()

        self._thread = None
        self._executor = None
        self._node = None
        self._status_publisher = None

        if self._started_rclpy:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
            self._started_rclpy = False

    def wait_for_start(self) -> bool:
        """Block while idle until Isaac requests a new episode or Esc stops us."""
        while not self._events["stop_recording"]:
            with self._condition:
                if self._start_requested:
                    self._start_requested = False
                    self._active_episode = True
                    self._end_command = None
                    return True
                self._condition.wait(timeout=0.1)
        return False

    def finish_episode(self):
        """Mark the episode inactive and return its ROS end command, if any."""
        with self._lock:
            self._active_episode = False
            command = self._end_command
            self._end_command = None
            return command

    def publish_status(self, status: str) -> None:
        if self._status_publisher is None:
            raise RuntimeError("Episode command bridge has not been started.")

        from std_msgs.msg import String

        message = String()
        message.data = status
        self._status_publisher.publish(message)
        logging.info("Published episode status %r.", status)

    def _on_command(self, message) -> None:
        command = message.data.strip()
        if command == self._cfg.start_command:
            with self._condition:
                if self._active_episode or self._start_requested:
                    logging.warning("Ignoring %r because an episode is already active or queued.", command)
                    return
                self._start_requested = True
                self._condition.notify_all()
            logging.info("Received %r from Isaac; beginning a new episode.", command)
            return

        if command not in (self._cfg.end_save_command, self._cfg.end_discard_command):
            logging.debug("Ignoring episode command %r.", command)
            return

        with self._lock:
            if not self._active_episode:
                logging.warning("Ignoring %r because no recording episode is active.", command)
                return
            if self._end_command is not None:
                logging.warning("Ignoring %r because an end command is already pending.", command)
                return
            self._end_command = command
            self._events["exit_early"] = True
        logging.info("Received %r from Isaac; ending the active episode.", command)


def _make_dataset(cfg: KaivuriRecordConfig, robot, dataset_features):
    if cfg.resume:
        num_cameras = len(robot.cameras) if hasattr(robot, "cameras") else 0
        dataset = LeRobotDataset.resume(
            cfg.dataset.repo_id,
            root=cfg.dataset.root,
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            rgb_encoder=cfg.dataset.rgb_encoder,
            depth_encoder=cfg.dataset.depth_encoder,
            encoder_threads=cfg.dataset.encoder_threads,
            streaming_encoding=cfg.dataset.streaming_encoding,
            encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
            image_writer_processes=cfg.dataset.num_image_writer_processes if num_cameras > 0 else 0,
            image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * num_cameras
            if num_cameras > 0
            else 0,
        )
        _check_resume_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
        return dataset

    repo_name = cfg.dataset.repo_id.split("/", 1)[-1]
    if repo_name.startswith("eval_"):
        raise ValueError("Dataset names starting with 'eval_' are reserved for lerobot-rollout.")
    if not cfg.dataset.repo_id:
        raise ValueError("--dataset.repo_id must be non-empty.")
    cfg.dataset.stamp_repo_id()
    return LeRobotDataset.create(
        cfg.dataset.repo_id,
        cfg.dataset.fps,
        root=cfg.dataset.root,
        robot_type=robot.name,
        features=dataset_features,
        use_videos=cfg.dataset.video,
        image_writer_processes=cfg.dataset.num_image_writer_processes,
        image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
        batch_encoding_size=cfg.dataset.video_encoding_batch_size,
        rgb_encoder=cfg.dataset.rgb_encoder,
        depth_encoder=cfg.dataset.depth_encoder,
        encoder_threads=cfg.dataset.encoder_threads,
        streaming_encoding=cfg.dataset.streaming_encoding,
        encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
    )


@parser.wrap()
def record(
    cfg: KaivuriRecordConfig,
    teleop_action_processor: RobotProcessorPipeline | None = None,
    robot_action_processor: RobotProcessorPipeline | None = None,
    robot_observation_processor: RobotProcessorPipeline | None = None,
) -> LeRobotDataset:
    """Run native recording with automatic Isaac success completion."""
    init_logging()
    if cfg.time_source not in ("simulation", "wall"):
        raise ValueError("time_source must be simulation or wall")
    cfg.robot.simulation_time = cfg.time_source == "simulation"
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        init_visualization(
            cfg.display_mode,
            session_name="recording",
            ip=cfg.display_ip,
            port=cfg.display_port,
        )
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop)
    if teleop_action_processor is None or robot_action_processor is None or robot_observation_processor is None:
        default_teleop, default_robot, default_observation = make_default_processors()
        teleop_action_processor = teleop_action_processor or default_teleop
        robot_action_processor = robot_action_processor or default_robot
        robot_observation_processor = robot_observation_processor or default_observation

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=upstream.create_initial_features(action=robot.action_features),
            use_videos=cfg.dataset.video,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=upstream.create_initial_features(observation=robot.observation_features),
            use_videos=cfg.dataset.video,
        ),
    )

    if cfg.robot.simulation_time:
        for key in TIME_FEATURES:
            dataset_features[key] = {"dtype": "int64", "shape": (1,), "names": None}

    dataset = None
    listener = None
    bridge = None
    try:
        dataset = _make_dataset(cfg, robot, dataset_features)
        robot.connect()
        teleop.connect()
        listener, events = init_keyboard_listener() # TODO: Remove this listener and use the ROS bridge to stop recording instead.
        bridge = EpisodeCommandBridge(events, cfg)
        bridge.start()

        with VideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
                log_say(
                    f"Ready for episode {dataset.num_episodes}. Press LB to start, or Esc to exit.",
                    cfg.play_sounds,
                )
                if not bridge.wait_for_start():
                    break

                log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
                loop = simulation_record_loop if cfg.robot.simulation_time else upstream.record_loop
                try:
                    loop(
                        robot=robot, events=events, fps=cfg.dataset.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=teleop, dataset=dataset,
                        control_time_s=cfg.dataset.episode_time_s,
                        single_task=cfg.dataset.single_task,
                        display_data=cfg.display_data, display_mode=cfg.display_mode,
                        display_compressed_images=display_compressed_images,
                    )
                except (ClockReset, MissingSample) as exc:
                    logging.warning("Discarding incomplete simulation episode: %s", exc)
                    bridge.finish_episode()
                    events["exit_early"] = False
                    events["rerecord_episode"] = False
                    dataset.clear_episode_buffer()
                    bridge.publish_status(cfg.discarded_status)
                    continue
                end_command = bridge.finish_episode()

                # Esc is an exit, not an implicit save.  Leave no partial episode
                # in memory or on disk when the operator stops the recorder.
                if events["stop_recording"]:
                    dataset.clear_episode_buffer()
                    break

                # ROS end commands use LeRobot's ``exit_early`` event to end
                # record_loop. It must not remain set for the next LB start.
                events["exit_early"] = False

                if events["rerecord_episode"]:
                    log_say("Re-record episode", cfg.play_sounds)
                    events["rerecord_episode"] = False
                    dataset.clear_episode_buffer()
                    continue

                if end_command == cfg.end_discard_command:
                    dataset.clear_episode_buffer()
                    bridge.publish_status(cfg.discarded_status)
                    log_say("Discarded episode", cfg.play_sounds)
                    continue

                if not dataset.has_pending_frames():
                    logging.warning("Episode ended before any frames were recorded; discarding it.")
                    dataset.clear_episode_buffer()
                    if end_command == cfg.end_save_command:
                        bridge.publish_status(cfg.discarded_status)
                    continue

                dataset.save_episode()
                recorded_episodes += 1

                if end_command == cfg.end_save_command:
                    bridge.publish_status(cfg.saved_status)
    finally:
        log_say("Stop recording", cfg.play_sounds, blocking=True)
        if dataset:
            dataset.finalize()
        if bridge:
            bridge.stop()
        if robot.is_connected:
            robot.disconnect()
        if teleop and teleop.is_connected:
            teleop.disconnect()
        if listener:
            listener.stop()
        if cfg.display_data:
            shutdown_visualization(cfg.display_mode)
        if cfg.dataset.push_to_hub:
            if dataset and dataset.num_episodes > 0:
                dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
            else:
                logging.warning("No episodes saved — skipping push to hub")
        log_say("Exiting", cfg.play_sounds)
    return dataset


def main() -> None:
    register_third_party_plugins()
    record()


if __name__ == "__main__":
    main()
