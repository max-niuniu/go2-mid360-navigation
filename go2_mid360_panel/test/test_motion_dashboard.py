import math
import time

import pytest
from nav_msgs.msg import Odometry

from go2_mid360_panel.motion_dashboard import MotionState, motion_label


def odom(vx=0., vy=0., wz=0., yaw=0., stamp=1):
    msg = Odometry()
    msg.header.frame_id = 'world'; msg.header.stamp.sec = stamp
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.twist.twist.linear.x = vx; msg.twist.twist.linear.y = vy
    msg.twist.twist.angular.z = wz
    return msg


@pytest.mark.parametrize('velocity,label', [
    ((0., 0., 0.), '静止'), ((.4, 0., 0.), '前进'), ((-.4, 0., 0.), '后退'),
    ((0., .3, 0.), '左移'), ((0., -.3, 0.), '右移'),
    ((0., 0., .5), '原地左旋'), ((0., 0., -.5), '原地右旋'),
    ((.4, 0., .3), '前进 · 左转'), ((-.4, 0., -.3), '后退 · 右转'),
])
def test_motion_direction(velocity, label):
    assert motion_label(*velocity) == label


def test_world_velocity_rotated_into_robot_heading():
    state = MotionState()
    state.receive(odom(vy=.5, yaw=math.pi/2))
    assert state.snapshot() == pytest.approx((.5, 0., 0.))
    state.receive(odom(vy=-.5, yaw=math.pi/2, stamp=3))
    assert motion_label(*state.snapshot()) == '后退'


def test_stale_invalid_and_repeated_data_not_shown_as_stopped():
    state = MotionState()
    assert state.snapshot() is None
    state.receive(odom(vx=.5))
    state.seen = time.monotonic() - 2
    state.receive(odom(vx=.5))
    assert state.snapshot() is None
    state.receive(odom(vx=float('nan'), stamp=2))
    assert state.snapshot() is None
    msg = odom(stamp=2); msg.header.frame_id = 'odom'
    state.receive(msg)
    assert state.snapshot() is None
    state.receive(odom(stamp=0))  # Gazebo reset, including a zero simulation timestamp.
    assert state.snapshot() == (0., 0., 0.)


def test_display_filter_suppresses_noise_and_tracks_reversal():
    state = MotionState()
    state.receive(odom(vx=.01, vy=-.01, wz=.01))
    assert state.snapshot() == (0., 0., 0.)
    state.receive(odom(vx=.5, stamp=3))
    for i in range(1, 4):
        msg = odom(vx=-.5, stamp=3); msg.header.stamp.nanosec = i*100000000
        state.receive(msg)
    assert state.snapshot()[0] < -.3
    assert motion_label(*state.snapshot()) == '后退'
