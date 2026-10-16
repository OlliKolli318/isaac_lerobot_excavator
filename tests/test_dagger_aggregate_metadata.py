"""Offline Parquet/LeRobot regressions for repeated DAgger aggregation."""
from pathlib import Path
import hashlib
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'lerobot_robot_isaac_excavator'))
from lerobot.datasets import LeRobotDataset
from lerobot_robot_isaac_excavator import dagger_aggregate
from lerobot_robot_isaac_excavator.episode_metadata import (
    LOCATION_COLUMNS, repair_episode_metadata_locations, validate_episode_metadata)


def files_digest(root):
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in root.rglob('*') if path.is_file()}


def create_dataset(root, labels):
    features = {'observation.state': {'dtype': 'float32', 'shape': (1,), 'names': ['joint']},
                'action': {'dtype': 'float32', 'shape': (1,), 'names': ['axis']}}
    if labels[0] is not None:
        features['intervention'] = {'dtype': 'bool', 'shape': (1,), 'names': None}
    dataset = LeRobotDataset.create('test/' + root.name, 30, root=root,
                                   features=features, robot_type='isaac_excavator', use_videos=False)
    try:
        for label in labels:
            for frame in range(2):
                row = {'observation.state': np.array([0.2], dtype=np.float32),
                       'action': np.array([0.8], dtype=np.float32), 'task': 'Pick up log'}
                if label is not None:
                    row['intervention'] = np.array([label], dtype=bool)
                dataset.add_frame(row)
            dataset.save_episode()
    finally:
        dataset.finalize()


class MetadataRepairTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='episode-metadata-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.dataset = self.root / 'dataset'
        create_dataset(self.dataset, [True, True])
        self.path = self.dataset / 'meta/episodes/chunk-000/file-000.parquet'

    def corrupt_location(self):
        table = pq.read_table(self.path)
        index = table.schema.get_field_index(LOCATION_COLUMNS[1])
        table = table.set_column(index, table.schema.field(index),
                                 pa.array([0, 4], type=table.schema.field(index).type))
        pq.write_table(table, self.path)
        return table

    def test_repair_backups_and_preserves_every_other_column(self):
        original = self.corrupt_location()
        original_bytes = self.path.read_bytes()
        data_before = files_digest(self.dataset / 'data')
        with self.assertRaisesRegex(ValueError, 'backup_root'):
            repair_episode_metadata_locations(self.dataset)
        self.assertEqual(self.path.read_bytes(), original_bytes)
        backup = self.root / 'backup'
        self.assertEqual(repair_episode_metadata_locations(self.dataset, backup_root=backup), 1)
        self.assertEqual((backup / 'meta/episodes/chunk-000/file-000.parquet').read_bytes(), original_bytes)
        fixed = pq.read_table(self.path)
        self.assertTrue(fixed.schema.equals(original.schema, check_metadata=True))
        for column in original.column_names:
            if column not in LOCATION_COLUMNS:
                self.assertTrue(fixed[column].equals(original[column]), column)
        self.assertEqual(fixed[LOCATION_COLUMNS[1]].to_pylist(), [0, 0])
        self.assertEqual(files_digest(self.dataset / 'data'), data_before)
        self.assertEqual(validate_episode_metadata(self.dataset), 0)
        before = files_digest(self.dataset)
        self.assertEqual(repair_episode_metadata_locations(self.dataset), 0)
        self.assertEqual(files_digest(self.dataset), before)

    def test_missing_episode_is_not_hidden_by_repair(self):
        table = self.corrupt_location()
        pq.write_table(table.slice(0, 1), self.path)
        before = files_digest(self.dataset)
        with self.assertRaisesRegex(ValueError, 'episode metadata rows'):
            repair_episode_metadata_locations(self.dataset, backup_root=self.root / 'backup')
        self.assertEqual(files_digest(self.dataset), before)
        self.assertFalse((self.root / 'backup').exists())

    def test_input_mismatch_is_rejected_before_output_creation(self):
        self.corrupt_location()
        correction = self.root / 'correction'
        create_dataset(correction, [True])
        with self.assertRaisesRegex(ValueError, 'wrong metadata file'):
            dagger_aggregate.aggregate(self.dataset, [correction], self.root / 'output', 'test/output')
        self.assertFalse((self.root / 'output').exists())


class AggregationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='dagger-aggregate-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_multiple_metadata_files_and_merge_of_previous_merge(self):
        demo, corrections, later = [self.root / name for name in ('demo', 'corrections', 'later')]
        create_dataset(demo, [None, None])
        create_dataset(corrections, [True, True])
        create_dataset(later, [True])
        source_path = corrections / 'meta/episodes/chunk-000/file-000.parquet'
        table = pq.read_table(source_path)
        second = table.slice(1, 1)
        for key, value in zip(LOCATION_COLUMNS, (1, 3), strict=True):
            index = second.schema.get_field_index(key)
            second = second.set_column(index, second.schema.field(index),
                                       pa.array([value], type=second.schema.field(index).type))
        second_path = corrections / 'meta/episodes/chunk-001/file-003.parquet'
        second_path.parent.mkdir(parents=True)
        pq.write_table(second, second_path)
        pq.write_table(table.slice(0, 1), source_path)
        validate_episode_metadata(corrections)
        before = {path: files_digest(path) for path in (demo, corrections, later)}

        first = dagger_aggregate.aggregate(demo, [corrections], self.root / 'first', 'test/first')
        self.assertEqual((first.num_episodes, first.num_frames), (4, 8))
        validate_episode_metadata(first.root)
        first_before = files_digest(first.root)
        second = dagger_aggregate.aggregate(first.root, [later], self.root / 'second', 'test/second')
        self.assertEqual((second.num_episodes, second.num_frames), (5, 10))
        validate_episode_metadata(second.root)
        rows = second.hf_dataset.select_columns(['intervention']).with_format(None)
        self.assertEqual([bool(np.asarray(row['intervention']).item()) for row in rows],
                         [False] * 4 + [True] * 6)
        self.assertEqual(files_digest(first.root), first_before)
        for path, digest in before.items():
            self.assertEqual(files_digest(path), digest)

    def test_failure_does_not_publish_a_partial_output(self):
        demo, correction = self.root / 'demo', self.root / 'correction'
        create_dataset(demo, [True])
        create_dataset(correction, [True])

        def fail_after_writing(**kwargs):
            staging = kwargs['aggr_root']
            staging.mkdir()
            (staging / 'partial').write_text('incomplete')
            raise RuntimeError('simulated merge failure')

        with patch.object(dagger_aggregate, 'aggregate_datasets', side_effect=fail_after_writing):
            with self.assertRaisesRegex(RuntimeError, 'simulated merge failure'):
                dagger_aggregate.aggregate(demo, [correction], self.root / 'output', 'test/output')
        self.assertFalse((self.root / 'output').exists())
        self.assertFalse(list(self.root.glob('dagger-aggregate-*')))

    def test_policy_episodes_are_filtered_and_existing_output_is_preserved(self):
        demo, correction = self.root / 'demo', self.root / 'correction'
        create_dataset(demo, [True])
        create_dataset(correction, [False, True, True])
        result = dagger_aggregate.aggregate(demo, [correction], self.root / 'output',
                                            'test/output', excluded_episodes=[2])
        self.assertEqual((result.num_episodes, result.num_frames), (2, 4))
        validate_episode_metadata(result.root)
        before = files_digest(result.root)
        with self.assertRaisesRegex(ValueError, 'new directory'):
            dagger_aggregate.aggregate(demo, [correction], result.root, 'test/overwrite')
        self.assertEqual(files_digest(result.root), before)


if __name__ == '__main__':
    unittest.main()
