"""Restart only an owned RViz process, and follow its launch lifetime."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def process_info(pid):
    try:
        path = Path('/proc') / str(pid)
        stat = (path / 'stat').read_text().rsplit(')', 1)[1].split()
        args = (path / 'cmdline').read_bytes().decode().strip('\0').split('\0')
        return stat[0], int(stat[2]), stat[19], args
    except (OSError, ValueError, IndexError):
        return None


def owned_rviz(owner_pid):
    found = []
    for entry in Path('/proc').glob('[0-9]*'):
        info = process_info(int(entry.name))
        if info and info[0] != 'Z' and info[1] == owner_pid and Path(info[3][0]).name == 'rviz2':
            found.append((int(entry.name), info[2], info[3]))
    return found


def same_process(pid, stamp):
    info = process_info(pid)
    return bool(info and info[0] != 'Z' and info[2] == stamp)


def main():
    owner = int(sys.argv[1])
    args = sys.argv[2:]
    info = process_info(owner)
    if not info or not args or Path(args[0]).name != 'rviz2':
        return 1
    owner_stamp = info[2]
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    for pid, stamp, _ in owned_rviz(owner):
        for sig, wait in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
            if not same_process(pid, stamp):
                break
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                break
            end = time.monotonic() + wait
            while same_process(pid, stamp) and time.monotonic() < end:
                time.sleep(.1)
        if same_process(pid, stamp):
            print('旧 RViz 尚未退出，未启动重复窗口。', flush=True)
            return 1
    if not running or not same_process(owner, owner_stamp):
        return 0
    print('正在重新打开 RViz；仿真与导航保持运行。', flush=True)
    child = subprocess.Popen(args)
    try:
        while running and same_process(owner, owner_stamp) and child.poll() is None:
            time.sleep(.2)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait()
    return 0


if __name__ == '__main__':
    sys.exit(main())
