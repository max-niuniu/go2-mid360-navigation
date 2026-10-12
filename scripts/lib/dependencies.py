#!/usr/bin/env python3
"""Prepare pinned sources and apply the project's compatibility patches."""
import argparse
from pathlib import Path
import re
import subprocess
import sys
import time

import yaml

REPO = Path(__file__).resolve().parents[2]
WORKSPACE = REPO.parent.parent
SOURCES = WORKSPACE / 'src' / 'third_party'
PATCHES = {
    'champ': 'champ.patch',
    'livox_ros_driver2': 'livox_ros_driver2.patch',
    'livox_laser_simulation_ros2': 'livox_laser_simulation_ros2.patch',
    'FAST_LIO_ROS2': 'fast_lio_ros2.patch',
}
SPARSE = {
    'geometry2': ['tf2'],
    'champ': ['champ', 'champ_base', 'champ_bringup', 'champ_config',
              'champ_description', 'champ_gazebo', 'champ_msgs'],
}


def git(path, *args, check=True):
    return subprocess.run(['git', '-C', str(path), *args], check=check,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def retry_git(path, *args):
    for attempt in range(3):
        result = git(path, '-c', 'http.version=HTTP/1.1', *args, check=False)
        if result.returncode == 0:
            return
        if attempt < 2:
            print(f'{path.name}: network operation failed; retrying...', flush=True)
            time.sleep(3)
    raise RuntimeError(result.stderr.strip())


def verify_revision(path, revision):
    if not (path / '.git').exists():
        raise RuntimeError(f'{path}: not a managed Git checkout. Use a clean workspace.')
    actual = git(path, 'rev-parse', '--verify', 'HEAD').stdout.strip()
    if actual != revision:
        raise RuntimeError(f'{path.name}: expected {revision}, found {actual}; files left unchanged.')


def verify_submodules(path, initialize=False):
    state = git(path, 'submodule', 'status', '--recursive').stdout
    if any(line.startswith(('+', 'U')) for line in state.splitlines()):
        raise RuntimeError(f'{path.name}: submodule revision mismatch; files left unchanged.')
    if any(line.startswith('-') for line in state.splitlines()):
        if not initialize:
            raise RuntimeError(f'{path.name}: uninitialized submodule; run prepare_sources.bash.')
        retry_git(path, 'submodule', 'update', '--init', '--recursive', '--depth=1')
        verify_submodules(path)


def prepare(name, spec):
    print(f'{name}: preparing pinned source...', flush=True)
    path = SOURCES / name
    if path.exists() and not (path / '.git').is_dir():
        raise RuntimeError(f'{path}: existing unmanaged directory; refusing to replace it.')
    if not path.exists():
        path.mkdir(parents=True)
        git(path, 'init', '-q')
        git(path, 'remote', 'add', 'origin', spec['url'])
    head = git(path, 'rev-parse', '--verify', 'HEAD', check=False)
    if head.returncode:
        if git(path, 'status', '--porcelain', '--untracked-files=all').stdout:
            raise RuntimeError(f'{path}: incomplete checkout contains files; refusing to overwrite.')
        retry_git(path, 'fetch', '--depth=1', 'origin', spec['version'])
        if name in SPARSE:
            git(path, 'sparse-checkout', 'set', *SPARSE[name])
        git(path, 'checkout', '--detach', spec['version'])
    verify_revision(path, spec['version'])
    verify_submodules(path, initialize=True)
    print(f'{name}: {spec["version"]}', flush=True)


def patch_plan(path, patch):
    # Check every touched file before writing; this also supports partial retries.
    names = re.findall(r'^diff --git a/(\S+) b/\S+$', patch.read_text(), re.MULTILINE)
    if not names:
        raise RuntimeError(f'Empty patch: {patch}')
    plan = []
    for name in names:
        args = ['apply', '--include=' + name]
        if git(path, *args, '--reverse', '--check', str(patch), check=False).returncode == 0:
            continue
        result = git(path, *args, '--check', str(patch), check=False)
        if result.returncode:
            raise RuntimeError(f'{path.name}/{name}: patch conflicts with local changes.\n{result.stderr}')
        plan.append((path, args + [str(patch)]))
    return plan


def apply(repositories):
    plan = []
    for name, spec in repositories.items():
        path = SOURCES / name
        verify_revision(path, spec['version'])
        verify_submodules(path)
        if name in PATCHES:
            plan.extend(patch_plan(path, REPO / 'patches' / PATCHES[name]))
    generated = [
        (SOURCES / 'livox_ros_driver2/package_ROS2.xml', SOURCES / 'livox_ros_driver2/package.xml'),
        (REPO / 'patches/tf2.colcon.pkg', SOURCES / 'geometry2/tf2/colcon.pkg'),
    ]
    for source, target in generated:
        if target.exists() and target.read_bytes() != source.read_bytes():
            raise RuntimeError(f'{target}: existing file differs; refusing to overwrite.')
    maps = REPO / 'go2_mid360_bringup/maps/go2_workshop'
    for name in ['map.pcd', 'map.pgm', 'map.yaml']:
        if not (maps / name).is_file():
            raise RuntimeError(f'Missing reference map: {maps / name}')
    for path, args in plan:
        git(path, *args)
    for source, target in generated:
        if not target.exists():
            target.write_bytes(source.read_bytes())
    target_maps = WORKSPACE / 'maps/go2_workshop'
    target_maps.mkdir(parents=True, exist_ok=True)
    for name in ['map.pcd', 'map.pgm', 'map.yaml']:
        target = target_maps / name
        if not target.exists():
            target.write_bytes((maps / name).read_bytes())
    print('Compatibility patches and reference maps are ready.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'apply'])
    args = parser.parse_args()
    repositories = yaml.safe_load((REPO / 'dependencies.repos').read_text())['repositories']
    for name, spec in repositories.items():
        if not re.fullmatch(r'[A-Za-z0-9_-]+', name) or not re.fullmatch(r'[0-9a-f]{40}', spec['version']):
            raise RuntimeError(f'Invalid pinned dependency: {name}')
        if spec['type'] != 'git':
            raise RuntimeError(f'Unsupported dependency type: {name}')
    if args.action == 'prepare':
        for name, spec in repositories.items():
            prepare(name, spec)
    apply(repositories)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, OSError) as error:
        detail = error.stderr if isinstance(error, subprocess.CalledProcessError) else str(error)
        print(f'ERROR: {detail}', file=sys.stderr)
        sys.exit(1)
