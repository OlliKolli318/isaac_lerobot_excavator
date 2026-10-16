"""ROS-free checks for the excavator's atomic simulation sample timeline."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lerobot_robot_isaac_excavator"))
from simulation_time import SimRate, SimTimeline


class TimelineTests(unittest.TestCase):
    def test_grid_no_drift(self):
        rate = SimRate(30)
        self.assertEqual(rate.at(30 * 3600), 3600_000_000_000)
        self.assertEqual(rate.after(rate.at(7)), 8)

    def test_atomic_sample(self):
        timeline = SimTimeline()
        timeline.put("sample", 100, ((0.0,) * 5, (1.0,) * 5, "run1"), "run1")
        timeline.put("camera2", 100, "cabin")
        timeline.put("camera3", 100, "gripper")
        timeline.update_clock(100)
        sample = timeline.wait_sample(100, timeline.epoch)
        self.assertEqual(sample.action, (1.0,) * 5)
        self.assertEqual(sample.images, {"camera2": "cabin", "camera3": "gripper"})
