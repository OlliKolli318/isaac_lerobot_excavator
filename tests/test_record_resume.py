"""Resume schema and frame regressions without the Windows ROS runtime."""
import ast
from copy import deepcopy
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1] / "lerobot_robot_isaac_excavator"


def load_functions(filename, names, namespace):
    # Exercise the actual functions while avoiding ROS/LeRobot import side effects.
    tree = ast.parse((PACKAGE / filename).read_text())
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), filename, "exec"), namespace)
    return namespace


TIME_FEATURES = ("clock_time_ns", "sim_time_ns", "camera2_time_ns", "camera3_time_ns")
DEFAULT_FEATURES = {
    key: {"dtype": "float32" if key == "timestamp" else "int64", "shape": (1,), "names": None}
    for key in ("timestamp", "frame_index", "episode_index", "index", "task_index")
}
record = load_functions("kaivuri_record.py", {"_without_feature_info", "_check_resume_compatibility"},
                        {"TIME_FEATURES": TIME_FEATURES})


class ResumeTests(unittest.TestCase):
    def setUp(self):
        self.features = {
            "action": {"dtype": "float32", "shape": (2,), "names": ["slew", "arm"]},
            "observation.images.camera2": {
                "dtype": "video", "shape": (8, 8, 3), "names": ["height", "width", "channels"]},
            **{key: {"dtype": "int64", "shape": (1,), "names": None} for key in TIME_FEATURES},
        }
        self.dataset = NS(features=deepcopy({**self.features, **DEFAULT_FEATURES}),
                          fps=30, meta=NS(robot_type="isaac_excavator"))
        self.robot = NS(robot_type="isaac_excavator")
        constants = ModuleType("lerobot.utils.constants")
        constants.DEFAULT_FEATURES = DEFAULT_FEATURES
        patcher = patch.dict(sys.modules, {"lerobot.utils.constants": constants})
        patcher.start()
        self.addCleanup(patcher.stop)

    def check(self, features=None):
        record["_check_resume_compatibility"](
            self.dataset, self.robot, 30, self.features if features is None else features)

    def add_label(self):
        self.dataset.features["intervention"] = {"dtype": "bool", "shape": [1], "names": None}

    def test_original_schema_and_json_codec_metadata(self):
        for feature in self.dataset.features.values():
            feature["shape"] = list(feature["shape"])
        self.dataset.features["observation.images.camera2"]["info"] = {"video.codec": "av1"}
        self.check()

    def test_aggregate_label_is_supported_without_changing_metadata(self):
        self.add_label()
        before = deepcopy(self.dataset.features)
        self.check()
        self.assertEqual(self.dataset.features, before)
        self.assertNotIn("intervention", self.features)

    def test_invalid_label_schema_is_rejected(self):
        for field, value in (("dtype", "float32"), ("shape", [2]), ("names", ["different"])):
            with self.subTest(field=field):
                self.add_label()
                self.dataset.features["intervention"][field] = value
                with self.assertRaisesRegex(ValueError, "features.intervention: expected"):
                    self.check()

    def test_robot_schema_changes_are_still_rejected(self):
        self.add_label()
        for key, field, value in (("action", "names", ["arm", "slew"]),
                                  ("observation.images.camera2", "shape", [16, 16, 3])):
            with self.subTest(key=key):
                features = deepcopy(self.features)
                features[key][field] = value
                with self.assertRaisesRegex(ValueError, f"features.{key}: expected"):
                    self.check(features)

    def test_unknown_extra_and_missing_feature_are_reported(self):
        self.dataset.features["unknown"] = {"dtype": "bool", "shape": [1], "names": None}
        del self.dataset.features["clock_time_ns"]
        with self.assertRaises(ValueError) as error:
            self.check()
        self.assertIn("features.unknown: unsupported extra", str(error.exception))
        self.assertIn("features.clock_time_ns: missing", str(error.exception))

    def test_wall_recording_does_not_accept_label_it_cannot_write(self):
        self.add_label()
        for key in TIME_FEATURES:
            del self.dataset.features[key]
        with self.assertRaisesRegex(ValueError, "features.intervention: unsupported extra"):
            self.check({key: value for key, value in self.features.items() if key not in TIME_FEATURES})

    def test_robot_type_and_fps_remain_strict(self):
        self.dataset.meta.robot_type = "other"
        self.dataset.fps = 60
        with self.assertRaises(ValueError) as error:
            self.check()
        self.assertIn("robot_type:", str(error.exception))
        self.assertIn("fps:", str(error.exception))

    def test_manual_frames_get_false_label_only_when_required(self):
        # Use the actual simulation clock and recording loop with synthetic samples.
        clock = {}
        exec(compile((PACKAGE / "simulation_time.py").read_text(), "simulation_time.py", "exec"), clock)
        loop = load_functions("simulation_record.py", {"simulation_record_loop"}, {
            "np": np, "TIME_FEATURES": TIME_FEATURES, "SimRate": clock["SimRate"],
            "MissingSample": clock["MissingSample"],
            "build_dataset_frame": lambda features, data, prefix: {prefix: data},
        })["simulation_record_loop"]

        class Timeline:
            def time(self):
                return 0, 0

            def wait_sample(self, ns, epoch, stopped):
                return NS(clock_ns=ns, stamp_ns=ns, image_stamps={"camera2": ns, "camera3": ns})

        robot = NS(timeline=Timeline(), observation_from_sample=lambda sample: 0,
                   action_from_sample=lambda sample: 1)
        for labelled in (False, True):
            with self.subTest(labelled=labelled):
                frames = []
                dataset = NS(features={"intervention": {}} if labelled else {}, add_frame=frames.append)
                loop(robot, {"stop_recording": False, "exit_early": False}, 30, dataset, .1,
                     "Pick and place", lambda obs: obs, lambda pair: pair[0])
                self.assertEqual(len(frames), 3)
                for frame in frames:
                    self.assertEqual("intervention" in frame, labelled)
                    if labelled:
                        self.assertEqual(frame["intervention"].dtype, np.dtype(bool))
                        np.testing.assert_array_equal(frame["intervention"], [False])


if __name__ == "__main__":
    unittest.main()
