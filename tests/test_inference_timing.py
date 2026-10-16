"""Timing regressions without requiring the Windows ROS/LeRobot environment."""
import ast
from collections import deque
from contextlib import nullcontext
import logging
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[1] / 'lerobot_robot_isaac_excavator'
sys.path.insert(0, str(SOURCE))
from inference_timing import InferenceTiming
from simulation_time import TimedChunk


class TimingTests(unittest.TestCase):
    def test_warmup_and_summary(self):
        timing = InferenceTiming()
        for _ in range(5):
            timing.record(9999, 9999)
        with self.assertLogs('inference_timing', level='INFO') as logs:
            for value in range(1, 21):
                timing.record(value, value * 2)
        self.assertEqual(len(timing.samples), 20)
        self.assertIn('model mean=10.50 median=10.50 p95=19.00', logs.output[-1])
        self.assertIn('pipeline mean=21.00 median=21.00 p95=38.00', logs.output[-1])

    def test_bounded_window_and_empty_report(self):
        timing = InferenceTiming()
        with self.assertLogs('inference_timing', level='INFO') as logs:
            timing.report()
        self.assertIn('no post-warm-up samples', logs.output[-1])
        for value in range(1006):
            timing.record(value, value)
        self.assertEqual(len(timing.samples), 1000)
        self.assertEqual(timing.samples[0], (6, 6))

    def test_policy_boundaries_cpu_cuda_and_disabled(self):
        # Execute the actual policy class with lightweight dependency doubles.
        tree = ast.parse((SOURCE / 'simulation_policy.py').read_text())
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
        for device_type, enabled in [('cpu', True), ('cuda', True), ('cuda', False)]:
            with self.subTest(device=device_type, enabled=enabled):
                events = []
                clock = Mock(side_effect=[10., 10.01, 10.03, 10.05])
                device = NS(type=device_type)
                sync = Mock(side_effect=lambda _: events.append('sync'))
                torch = NS(device=lambda _: device, cuda=NS(synchronize=sync),
                           no_grad=nullcontext, autocast=lambda **_: nullcontext())
                namespace = dict(deque=deque, threading=threading, math=math,
                                 time=NS(perf_counter=clock), torch=torch,
                                 nullcontext=nullcontext, TimedChunk=TimedChunk,
                                 InferenceTiming=InferenceTiming, logger=logging.getLogger('test'),
                                 build_dataset_frame=lambda *a, **k: {},
                                 prepare_observation_for_inference=lambda batch, *a: batch)
                exec(compile(ast.Module(body=[cls], type_ignores=[]), 'simulation_policy.py', 'exec'), namespace)

                class Tensor:
                    def squeeze(self, *a): return self
                    def detach(self): return self
                    def clone(self): return self
                    def cpu(self):
                        events.append('cpu')
                        return self
                    def __getitem__(self, key): return self
                    def __len__(self): return 10

                def forward(*a, **k):
                    events.append('forward')
                    return Tensor()

                processor = Mock(side_effect=lambda value: value)
                processor.steps = []
                cfg = NS(inference=NS(type='sync'), device=device_type, fps=30,
                         task='test', measure_inference_time=enabled)
                policy = Mock()
                policy.config = NS(type='smolvla', device=device_type, use_amp=False, n_action_steps=10)
                policy.predict_action_chunk.side_effect = forward
                ctx = NS(runtime=NS(cfg=cfg), data=NS(hw_features={}),
                         hardware=NS(robot_wrapper=NS(inner=NS(timeline=NS(time=lambda: (0, 0))), robot_type='test')),
                         policy=NS(policy=policy, preprocessor=processor, postprocessor=processor))
                engine = namespace['SimulationPolicy'](ctx)
                if enabled:
                    engine.timing = Mock()
                engine.predict(({}, 0, 0))
                self.assertIsNotNone(engine.chunk)
                if enabled:
                    model_ms, pipeline_ms = engine.timing.record.call_args.args
                    self.assertAlmostEqual(model_ms, 20)
                    self.assertAlmostEqual(pipeline_ms, 50)
                    self.assertEqual(clock.call_count, 4)
                else:
                    clock.assert_not_called()
                self.assertEqual(events, ['sync', 'sync', 'forward', 'sync', 'cpu', 'sync']
                                 if device_type == 'cuda' and enabled else ['forward', 'cpu'])


if __name__ == '__main__':
    unittest.main()
