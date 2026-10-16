"""Build a new training dataset: trusted demonstrations + expert corrections.

Original datasets are read-only. Autonomous rollout segments are excluded;
mixed-label episodes are rejected rather than sliced across temporal gaps.
"""
import argparse
import logging
from pathlib import Path
import tempfile

import numpy as np
from lerobot.datasets import LeRobotDataset, add_features, split_dataset
from lerobot.datasets.aggregate import aggregate_datasets

from .episode_metadata import repair_episode_metadata_locations, validate_episode_metadata


def correction_episodes(dataset):
    if 'intervention' not in dataset.features:
        raise ValueError(f'{dataset.root}: missing intervention feature')
    labels = {}
    rows = dataset.hf_dataset.select_columns(['episode_index', 'intervention']).with_format(None)
    for row in rows:
        values = np.asarray(row['intervention']).reshape(-1)
        if len(values) != 1 or values[0] not in (True, False):
            raise ValueError('Invalid intervention label')
        labels.setdefault(int(row['episode_index']), set()).add(bool(values[0]))
    if any(len(values) != 1 for values in labels.values()):
        raise ValueError('Mixed expert/policy episode found; cannot remove frames without breaking the simulation-time grid.')
    result = sorted(index for index, values in labels.items() if values == {True})
    if not result:
        raise ValueError(f'{dataset.root}: no expert correction episodes to aggregate')
    return result


def validate_selected_video_ranges(dataset, episode_indices):
    """Reject metadata whose claimed video duration cannot represent its frames.

    `split_dataset` otherwise reaches this condition later with an internal
    assertion after copying data. Checking first identifies the source episode
    and leaves the requested aggregate output untouched.
    """
    video_keys = list(dataset.meta.video_keys)
    if not video_keys:
        return
    rows = {int(row['episode_index']): row for row in dataset.meta.episodes}
    errors = []
    for index in episode_indices:
        row = rows[index]
        for key in video_keys:
            prefix = f'videos/{key}'
            start, end = float(row[f'{prefix}/from_timestamp']), float(row[f'{prefix}/to_timestamp'])
            represented = round((end - start) * dataset.meta.fps)
            if abs(represented - int(row['length'])) > 1:
                errors.append(f'episode {index}, {key}: {row["length"]} data frames but {represented} video frames')
    if errors:
        raise ValueError(
            'Correction video metadata is inconsistent; do not aggregate those episodes. ' +
            'Exclude or repair: ' + '; '.join(errors)
        )


def aggregate(demo_root, correction_roots, output_root, repo_id, excluded_episodes=()):
    demo_root, output_root = Path(demo_root).resolve(), Path(output_root).resolve()
    roots = [demo_root] + [Path(p).resolve() for p in correction_roots]
    if len(set(roots)) != len(roots):
        raise ValueError('A dataset was supplied twice; refusing to duplicate training data.')
    if output_root.exists() or any(p == output_root or p in output_root.parents or output_root in p.parents for p in roots):
        raise ValueError('Output must be a new directory separate from all input datasets.')
    if any(not (p / 'meta/info.json').is_file() for p in roots):
        raise ValueError('All input roots must be existing local LeRobot datasets; no automatic downloads.')
    for root in roots:
        validate_episode_metadata(root)
    excluded_episodes = tuple(sorted(set(int(index) for index in excluded_episodes)))
    if any(index < 0 for index in excluded_episodes):
        raise ValueError('--exclude-episode must be a non-negative correction episode index.')
    if excluded_episodes and len(correction_roots) != 1:
        raise ValueError('--exclude-episode is unambiguous only with one --corrections-root. Filter each collection separately before combining several.')
    datasets = [LeRobotDataset(f'local/dagger_source_{i}', root=p) for i, p in enumerate(roots)]
    selected = [correction_episodes(ds) for ds in datasets[1:]]
    if excluded_episodes:
        available = set(selected[0])
        unknown = sorted(set(excluded_episodes) - available)
        if unknown:
            raise ValueError(f'Cannot exclude correction episodes not selected as expert data: {unknown}')
        selected[0] = [index for index in selected[0] if index not in excluded_episodes]
        if not selected[0]:
            raise ValueError('All expert correction episodes were excluded; refusing to create an aggregate without corrections.')
    for dataset, episodes in zip(datasets[1:], selected, strict=True):
        validate_selected_video_ranges(dataset, episodes)
    output_root.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='dagger-aggregate-', dir=output_root.parent) as temporary:
        work = Path(temporary)
        demo = datasets[0]
        if 'intervention' not in demo.features:
            demo = add_features(demo, {'intervention': (
                np.zeros((demo.num_frames, 1), dtype=bool), {'dtype': 'bool', 'shape': (1,), 'names': None})},
                output_dir=work / 'demonstrations', repo_id='local/dagger_demonstrations')
        sources = [demo]
        for i, (dataset, episodes) in enumerate(zip(datasets[1:], selected, strict=True)):
            logging.info('Source %s: using %d of %d episodes (expert only)', dataset.root, len(episodes), dataset.num_episodes)
            if len(episodes) != dataset.num_episodes:
                dataset = split_dataset(dataset, {'expert': episodes}, output_dir=work / f'corrections_{i}')['expert']
            sources.append(dataset)
        # Use the same LeRobot aggregation backend as merge_datasets, but finish
        # its metadata before opening or publishing the resulting dataset. Some
        # versions retain source meta/episodes file indices after concatenating
        # metadata into fewer destination files, breaking the next aggregation.
        staged = work / 'merged'
        aggregate_datasets(
            repo_ids=[source.repo_id for source in sources], aggr_repo_id=repo_id,
            roots=[source.root for source in sources], aggr_root=staged,
            concatenate_videos=False, concatenate_data=False)
        repaired = repair_episode_metadata_locations(staged, new_output=True)
        validate_episode_metadata(staged)
        logging.info('Verified merged metadata; corrected %d file-location references', repaired)
        if output_root.exists():
            raise FileExistsError(f'Output appeared during aggregation: {output_root}')
        staged.rename(output_root)
        merged = LeRobotDataset(repo_id, root=output_root)
        logging.info('Training dataset ready: %s (%d episodes, %d frames)', output_root, merged.num_episodes, merged.num_frames)
    return merged


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo-root', required=True, help='Trusted expert demonstrations (or previous expert aggregate)')
    parser.add_argument('--corrections-root', action='append', required=True, help='Repeat for each NEW DAgger collection')
    parser.add_argument('--exclude-episode', type=int, action='append', default=[],
                        help='Repeat to omit expert episode indices from the one corrections dataset.')
    parser.add_argument('--output-root', required=True)
    parser.add_argument('--repo-id', required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    aggregate(args.demo_root, args.corrections_root, args.output_root, args.repo_id, args.exclude_episode)


if __name__ == '__main__':
    main()
