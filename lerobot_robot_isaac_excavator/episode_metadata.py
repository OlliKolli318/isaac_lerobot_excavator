"""Validate and repair LeRobot episode metadata's references to its own files.

Metadata may be consolidated independently of data/video files. The two
meta/episodes location columns must describe the physical containing parquet,
not the original source file. Data/video references and all other columns are
preserved. Repairing an existing dataset requires a separate metadata backup.
"""
import json
import os
from pathlib import Path
import shutil
import tempfile

import pyarrow as pa
import pyarrow.parquet as pq


LOCATION_COLUMNS = ('meta/episodes/chunk_index', 'meta/episodes/file_index')


def _episode_tables(root):
    root = Path(root)
    paths = sorted((root / 'meta/episodes').glob('chunk-*/file-*.parquet'))
    if not paths:
        raise ValueError(f'{root}: no episode metadata parquet files')
    return [(path, pq.read_table(path),
             (int(path.parent.name.removeprefix('chunk-')),
              int(path.stem.removeprefix('file-')))) for path in paths]


def validate_episode_metadata(root, *, require_locations=True):
    """Check episode coverage, frame ranges, file existence and self-locations.

This reads metadata only, without decoding videos or modifying a dataset.
Returns the number of rows with incorrect metadata-file locations.
"""
    root = Path(root)
    info = json.loads((root / 'meta/info.json').read_text(encoding='utf-8'))
    records = []
    mismatches = 0
    referenced = set()
    for path, table, location in _episode_tables(root):
        for row in table.to_pylist():
            if tuple(row.get(key) for key in LOCATION_COLUMNS) != location:
                mismatches += 1
            records.append(row)
            referenced.add(root / info['data_path'].format(
                chunk_index=row['data/chunk_index'], file_index=row['data/file_index']))
            for key, feature in info['features'].items():
                if feature['dtype'] == 'video':
                    referenced.add(root / info['video_path'].format(
                        video_key=key, chunk_index=row[f'videos/{key}/chunk_index'],
                        file_index=row[f'videos/{key}/file_index']))
    records.sort(key=lambda row: row['episode_index'])
    if [row['episode_index'] for row in records] != list(range(info['total_episodes'])):
        raise ValueError(f'{root}: missing, duplicated or unexpected episode metadata rows')
    offset = 0
    for row in records:
        if row['length'] <= 0 or (row['dataset_from_index'], row['dataset_to_index']) != (
                offset, offset + row['length']):
            raise ValueError(f'{root}: inconsistent frame range for episode {row["episode_index"]}')
        offset += row['length']
    if offset != info['total_frames']:
        raise ValueError(f'{root}: episode lengths do not match total_frames')
    missing = sorted(str(path) for path in referenced if not path.is_file())
    if missing:
        raise ValueError(f'{root}: missing referenced data/video files: {missing}')
    if require_locations and mismatches:
        raise ValueError(
            f'{root}: {mismatches} episode rows reference the wrong metadata file; '
            'repair their meta/episodes location columns before aggregating')
    return mismatches


def repair_episode_metadata_locations(root, *, backup_root=None, new_output=False):
    """Correct only metadata self-locations, preserving Arrow types and stats.

For existing data, copy all metadata to a new backup directory before changing
anything. new_output is only for unpublished, disposable aggregate output.
Each replacement is verified and atomic; repeated calls make no changes.
"""
    root = Path(root).resolve()
    mismatches = validate_episode_metadata(root, require_locations=False)
    if not mismatches:
        return 0
    if backup_root is None and not new_output:
        raise ValueError('Repairing existing metadata requires a new backup_root')
    if backup_root is not None:
        backup_root = Path(backup_root).resolve()
        if backup_root == root or root in backup_root.parents or backup_root in root.parents:
            raise ValueError('Metadata backup must be separate from the dataset')
        if backup_root.exists():
            raise FileExistsError(backup_root)
        shutil.copytree(root / 'meta', backup_root / 'meta')
    for path, table, location in _episode_tables(root):
        if all(key in table.column_names and
               all(value == target for value in table[key].to_pylist())
               for key, target in zip(LOCATION_COLUMNS, location, strict=True)):
            continue
        signature = path.stat()
        fixed = table
        for key, target in zip(LOCATION_COLUMNS, location, strict=True):
            if key not in table.column_names:
                raise ValueError(f'{path}: missing required column {key}')
            index = fixed.schema.get_field_index(key)
            field = fixed.schema.field(index)
            fixed = fixed.set_column(index, field, pa.array([target] * table.num_rows, type=field.type))
        with tempfile.NamedTemporaryFile(prefix='.episode-metadata-', suffix='.tmp',
                                         dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
        try:
            pq.write_table(fixed, temporary)
            checked = pq.read_table(temporary)
            if not checked.equals(fixed, check_metadata=True):
                raise ValueError(f'{path}: replacement failed Parquet round-trip verification')
            unchanged_columns = [key for key in table.column_names if key not in LOCATION_COLUMNS]
            if not checked.select(unchanged_columns).equals(table.select(unchanged_columns)):
                raise ValueError(f'{path}: repair would alter episode data or statistics')
            current = path.stat()
            if (current.st_size, current.st_mtime_ns) != (signature.st_size, signature.st_mtime_ns):
                raise RuntimeError(f'{path}: metadata changed during repair; stop collection first')
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
    validate_episode_metadata(root)
    return mismatches
