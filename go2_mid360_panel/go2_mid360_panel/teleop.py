"""Supervise the upstream keyboard controller and exclude navigation."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

NAV_NODES = {'controller_server', 'behavior_server', 'bt_navigator',
             'velocity_smoother', 'lifecycle_manager_navigation'}


def conflict_reason(node, own_name=None):
    if any(name in NAV_NODES for name, _ in node.get_node_names_and_namespaces()):
        return 'Navigation is running. Stop navigation before using keyboard control.'
    publishers = [info for info in node.get_publishers_info_by_topic('/cmd_vel')
                  if info.node_name != own_name]
    if publishers:
        names = sorted({info.node_namespace.rstrip('/') + '/' + info.node_name
                        for info in publishers})
        return 'Another velocity publisher is active (' + ', '.join(names) + '). Stop it before using keyboard control.'
    return ''


class PublisherGuard:
    """Wait for DDS endpoint/node association before enabling keyboard input."""
    def __init__(self, own_name):
        self.own_name = own_name
        self.own_gids = set()
        self.pending = {}

    def check(self, node):
        if any(name in NAV_NODES for name, _ in node.get_node_names_and_namespaces()):
            return 'Navigation is running. Stop navigation before using keyboard control.', False
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        own_present = False
        unknown = set()
        now = time.monotonic()
        for info in publishers:
            gid = bytes(info.endpoint_gid)
            if info.node_name == self.own_name:
                if gid and any(gid):
                    self.own_gids.add(gid)
                own_present = True
            elif gid in self.own_gids:
                own_present = True
            elif info.node_name == '_NODE_NAME_UNKNOWN_':
                unknown.add(gid)
            else:
                name = info.node_namespace.rstrip('/') + '/' + info.node_name
                return f'Another velocity publisher is active ({name}). Stop it before using keyboard control.', False
        self.pending = {gid: self.pending.get(gid, now) for gid in unknown}
        if any(now - first >= 2. for first in self.pending.values()):
            return 'Velocity publisher identification timed out. Keyboard control stopped. Please try again.', False
        return '', own_present and not unknown


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('go2_keyboard_' + str(os.getpid()))
        self.clock_seen = 0
        self.clock_stamp = None
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def clock(self, msg):
        stamp = (msg.clock.sec, msg.clock.nanosec)
        if stamp != self.clock_stamp:
            self.clock_seen = time.monotonic()
        self.clock_stamp = stamp


def main():
    rclpy.init()
    node = KeyboardNode()
    keyboard_name = 'go2_keyboard_input_' + str(os.getpid())
    terminal = None
    directory = Path(tempfile.mkdtemp(prefix='go2-keyboard-'))
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        started = time.monotonic()
        while running and time.monotonic() - started < 2:
            rclpy.spin_once(node, timeout_sec=.05)
        reason = conflict_reason(node)
        if reason:
            print(reason, flush=True)
            return 1
        if time.monotonic() - node.clock_seen > 2:
            print('Simulation clock is not ready. Keyboard control did not start.', flush=True)
            return 1
        if not running:
            return 0
        command = [sys.executable, str(Path(__file__).with_name('keyboard_input.py')),
                   '--session-dir', str(directory), '--node-name', keyboard_name, *sys.argv[1:]]
        # GNOME Terminal may reuse a server started before this ROS environment.
        ros_environment = ('PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
                           'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
                           'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                           'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML')
        command = ['env', *[key + '=' + os.environ[key] for key in ros_environment if key in os.environ], *command]
        if shutil.which('gnome-terminal'):
            command = ['gnome-terminal', '--wait', '--title=Go2 Keyboard Control', '--', *command]
        elif shutil.which('xterm'):
            command = ['xterm', '-T', 'Go2 Keyboard Control', '-e', *command]
        else:
            print('Install gnome-terminal to use keyboard control.', flush=True)
            return 1
        terminal = subprocess.Popen(command)
        started = time.monotonic()
        print('Opening the keyboard control terminal.', flush=True)
        ready = False
        unidentified_since = None
        guard = PublisherGuard(keyboard_name)
        enabled = directory / 'enabled'
        next_update = 0.0
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if now < next_update:
                continue
            next_update = now + .05
            reason, identified = guard.check(node)
            if identified:
                unidentified_since = None
                if not enabled.exists():
                    enabled.touch()
            else:
                if unidentified_since is None:
                    unidentified_since = now
                enabled.unlink(missing_ok=True)
            if reason:
                print('Keyboard control exited: ' + reason, flush=True)
                return 1
            if (directory / 'stop').exists() or terminal.poll() is not None:
                break
            if time.monotonic() - node.clock_seen > 2:
                print('Simulation clock stopped. Keyboard control stopped.', flush=True)
                break
            try:
                heartbeat = float((directory / 'heartbeat').read_text())
            except (OSError, ValueError):
                if time.monotonic() - started > 10:
                    print('Keyboard control terminal is not ready.', flush=True)
                    break
                continue
            now = time.monotonic()
            if now - heartbeat > 1:
                print('Terminal connection lost. Keyboard control stopped.', flush=True)
                break
            if unidentified_since is not None and now - unidentified_since > 10:
                print('Velocity publisher is not ready. Keyboard control stopped.', flush=True)
                break
            if not ready and identified:
                ready = True
                print('Keyboard control is ready.', flush=True)
        return 0
    finally:
        (directory / 'finished').touch()
        if terminal is not None:
            try:
                terminal.wait(timeout=2)
            except subprocess.TimeoutExpired:
                terminal.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        shutil.rmtree(directory, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
