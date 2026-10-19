"""Bounded wall-time statistics, independent of the simulation clock."""
from collections import deque
import logging
import math
import statistics
import threading

logger = logging.getLogger(__name__)


class InferenceTiming:
    """Skip five startup chunks; summarize the latest 1000 completed chunks."""

    def __init__(self):
        self.completed = 0
        self.samples: deque[tuple[float, float]] = deque(maxlen=1000)
        self.lock = threading.Lock()

    def record(self, model_ms, pipeline_ms):
        with self.lock:
            self.completed += 1
            completed = self.completed
            if completed > 5:
                self.samples.append((model_ms, pipeline_ms))
        if completed <= 5:
            logger.info("Inference warm-up %d/5: model=%.2f ms, pipeline=%.2f ms",
                        completed, model_ms, pipeline_ms)
        elif (completed - 5) % 20 == 0:
            self.report()

    def report(self):
        with self.lock:
            samples, completed = list(self.samples), self.completed
        if not samples:
            logger.info("Inference timing: %d completed chunks; no post-warm-up samples yet.", completed)
            return
        values: list[float] = []
        for column in zip(*samples):
            ordered = sorted(column)
            values.extend((statistics.mean(column), statistics.median(column),
                           ordered[math.ceil(0.95 * len(ordered)) - 1]))
        logger.info(
            "Inference timing (wall ms/chunk; completed=%d, window=%d, first 5 excluded): "
            "model mean=%.2f median=%.2f p95=%.2f; pipeline mean=%.2f median=%.2f p95=%.2f",
            completed, len(samples), *values)
