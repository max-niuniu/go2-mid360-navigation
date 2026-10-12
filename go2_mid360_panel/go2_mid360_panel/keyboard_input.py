"""WASD-area key mapping for the installed teleop_twist_keyboard program."""
import argparse
import os
from pathlib import Path
import select
import signal
import sys
import time
import tty

import teleop_twist_keyboard as upstream

MOVEMENT_KEYS = dict(zip('qweadzxcQWEADZXC', 'uiojlm,.UIOJLM<>'))
SPEED_KEYS = dict(zip('rftgyh', 'qzwxec'))
HELP = """
Go2 Keyboard Control

    Q Forward left    W Forward    E Forward right
    A Turn left       S Stop       D Turn right
    Z Backward left   X Backward   C Backward right

Shift + movement keys: strafe (Shift+A / D: left / right)
S / Space: stop    Ctrl+C: quit

R / F: increase / decrease both speeds by 10%
T / G: increase / decrease linear speed by 10%
Y / H: increase / decrease angular speed by 10%

Hold a movement key to move. Stops within 0.5 s after release.
"""


def configure_keys():
    upstream.moveBindings = {new: upstream.moveBindings[old] for new, old in MOVEMENT_KEYS.items()}
    upstream.speedBindings = {new: upstream.speedBindings[old] for new, old in SPEED_KEYS.items()}
    upstream.msg = HELP
    upstream.vels = lambda speed, turn: f'Linear speed: {speed:.3f} m/s    Angular speed: {turn:.3f} rad/s'


def main(directory=None, node_name=None, ros_args=()):
    configure_keys()
    running = True
    input_enabled = False
    def stop(*_):
        nonlocal running
        running = False
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)

    def get_key(settings):
        nonlocal input_enabled
        tty.setraw(sys.stdin.fileno())
        try:
            while running and directory.exists() and not (directory / 'finished').exists():
                temporary = directory / 'heartbeat.tmp'
                temporary.write_text(str(time.monotonic()))
                temporary.replace(directory / 'heartbeat')
                enabled = (directory / 'enabled').exists()
                if input_enabled and not enabled:
                    input_enabled = False
                    return ' '  # Stop once if a new endpoint needs identification.
                input_enabled = enabled
                if select.select([sys.stdin], [], [], .05)[0]:
                    key = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
                    if not key or key == '\x03':
                        return '\x03'
                    if enabled:
                        return key
            return '\x03'
        finally:
            upstream.restoreTerminalSettings(settings)

    if directory is not None:
        upstream.getKey = get_key
    sys.argv = [sys.argv[0], *ros_args]
    if node_name is not None:
        sys.argv += ['--ros-args', '-r', '__node:=' + node_name]
    try:
        upstream.main()
    finally:
        if directory is not None and directory.exists():
            (directory / 'stop').touch()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--session-dir', type=Path)
    parser.add_argument('--node-name')
    args, ros_args = parser.parse_known_args()
    main(args.session_dir, args.node_name, ros_args)
