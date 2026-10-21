"""ROS-free checks for the excavator's atomic simulation sample timeline."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lerobot_robot_isaac_excavator"))
from simulation_time import ClockReset, MissingSample, SimRate, SimTimeline


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

    def test_nearest_retains_tolerance_tie_and_out_of_order_behavior(self):
        timeline = SimTimeline(tolerance_s=2e-9)
        timeline.put("camera2", 102, "first")
        timeline.put("camera2", 98, "second")
        self.assertEqual(timeline._nearest("camera2", 100), 102)
        self.assertEqual(timeline._nearest("camera2", 99), 98)
        self.assertIsNone(timeline._nearest("camera2", 105))
        self.assertIsNone(timeline._nearest("camera3", 100))

    def test_reset_invalidates_episode_and_retires_old_run(self):
        timeline = SimTimeline()
        timeline.update_clock(100)
        timeline.put("sample", 100, (), "old")
        epoch = timeline.epoch
        timeline.update_clock(0)
        timeline.put("sample", 101, (), "old")
        self.assertIsNone(timeline.run_id)
        with self.assertRaises(ClockReset):
            timeline.wait_sample(100, epoch)
        timeline.put("sample", 0, (), "new")
        self.assertEqual(timeline.run_id, "new")

    def test_missing_samples_stop_after_simulation_grace(self):
        timeline = SimTimeline()
        timeline.update_clock(1_000_000_000)
        with self.assertRaises(MissingSample):
            timeline.wait_sample(0, timeline.epoch)
