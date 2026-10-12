"""Opt-in ROS/PTY regression: GO2_TELEOP_TEST_DOMAIN=<unused ROS domain>."""
import json
import os
import subprocess
import sys
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data


@pytest.mark.skipif(not os.environ.get('GO2_TELEOP_TEST_DOMAIN'),
                    reason='requires an unused ROS domain')
@pytest.mark.parametrize('anonymous_discovery', [False, True])
def test_key_repeat_does_not_delay_reverse_or_stop(tmp_path, monkeypatch, anonymous_discovery):
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ['GO2_TELEOP_TEST_DOMAIN'])
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    rclpy.init()
    observer = rclpy.create_node('teleop_latency_test')
    clock = observer.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    received = []
    observer.create_subscription(
        Twist, '/cmd_vel',
        lambda msg: received.append((time.monotonic(), msg.linear.x, msg.angular.z)), 100)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = time.monotonic()
            msg = Clock()
            msg.clock.sec = int(now)
            msg.clock.nanosec = int(now % 1 * 1e9)
            clock.publish(msg)
            rclpy.spin_once(observer, timeout_sec=.002)

    # Feed key repeats through a real PTY and the installed upstream keyboard program.
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    terminal = bindir / 'gnome-terminal'
    terminal.write_text('''#!/usr/bin/python3
import json, os, pty, select, subprocess, sys, time
from pathlib import Path
root = Path(os.environ['TELEOP_TEST_ROOT'])
master, slave = pty.openpty()
child = subprocess.Popen(sys.argv[sys.argv.index('--') + 1:],
                         stdin=slave, stdout=slave, stderr=slave)
os.close(slave)
try:
    while child.poll() is None:
        if select.select([master], [], [], .005)[0]:
            try: os.read(master, 65536)
            except OSError: break
        request = root / 'request'
        if request.exists():
            repeat, last = json.loads(request.read_text())
            request.unlink()
            for _ in range(90):
                os.write(master, repeat.encode())
                time.sleep(1 / 30)
            (root / 'sent').write_text(str(time.monotonic()))
            os.write(master, last.encode())
finally:
    os.close(master)
    if child.poll() is None:
        child.terminate()
        child.wait(timeout=5)
''')
    terminal.chmod(0o755)
    process = None
    try:
        spin(2)
        assert not observer.get_publishers_info_by_topic('/cmd_vel'), 'test domain is in use'
        assert len(observer.get_publishers_info_by_topic('/clock')) == 1, 'test domain is in use'
        command = [sys.executable, '-m', 'go2_mid360_panel.teleop']
        if anonymous_discovery:
            command = [sys.executable, '-c', """
import time
from types import SimpleNamespace
from go2_mid360_panel import teleop
original = teleop.KeyboardNode.get_publishers_info_by_topic
first = None
def delayed_names(self, topic):
    global first
    publishers = original(self, topic)
    if publishers and first is None:
        first = time.monotonic()
    age = time.monotonic() - first if first else 99
    if age < .4 or 5 < age < 5.4:
        return [SimpleNamespace(node_name='_NODE_NAME_UNKNOWN_',
                                node_namespace='_NODE_NAMESPACE_UNKNOWN_',
                                endpoint_gid=p.endpoint_gid) for p in publishers]
    return publishers
teleop.KeyboardNode.get_publishers_info_by_topic = delayed_names
raise SystemExit(teleop.main())
"""]
        process = subprocess.Popen(
            command,
            env={**os.environ, 'PATH': str(bindir) + ':' + os.environ['PATH'],
                 'TELEOP_TEST_ROOT': str(tmp_path)},
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic() + 10
        while not observer.get_publishers_info_by_topic('/cmd_vel') and time.monotonic() < deadline:
            spin(.05)
        publishers = observer.get_publishers_info_by_topic('/cmd_vel')
        assert [p.node_name for p in publishers] == ['go2_keyboard_input_' + str(process.pid)]
        spin(.3)
        for repeat, last, expected in [('w', 'x', (-.5, 0.)),
                                       ('x', ' ', (0., 0.)),
                                       ('a', 'd', (0., -1.))]:
            received.clear()
            (tmp_path / 'sent').unlink(missing_ok=True)
            request = tmp_path / 'request.tmp'
            request.write_text(json.dumps([repeat, last]))
            request.rename(tmp_path / 'request')
            spin(4.5)
            sent = float((tmp_path / 'sent').read_text())
            public = next(t for t, x, z in received if t >= sent and (x, z) == expected)
            delay = public - sent
            print(f'{repeat!r} -> {last!r}: cmd_vel={delay*1000:.1f} ms')
            assert delay < .2, f'key repeat delayed latest command by {delay:.3f} s'
            assert all((x, z) == expected for t, x, z in received if t > public)
            received.clear()
            spin(.5)
            assert not received, 'keyboard must not repeat stale commands when idle'
        external = observer.create_publisher(Twist, '/cmd_vel', 1)
        deadline = time.monotonic() + 5
        while process.poll() is None and time.monotonic() < deadline:
            spin(.05)
        assert process.poll() == 1, 'another publisher must stop the keyboard'
        observer.destroy_publisher(external)
    finally:
        if process is not None:
            if process.poll() is None:
                process.terminate()
            spin(.3)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            print(process.stdout.read())
        observer.destroy_node()
        rclpy.shutdown()
