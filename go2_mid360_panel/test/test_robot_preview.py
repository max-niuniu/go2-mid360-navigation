from pathlib import Path
import time

import numpy as np
import pytest
from PyQt5 import QtCore, QtWidgets
import rclpy
from sensor_msgs.msg import JointState

from go2_mid360_panel.robot_preview import PreviewModel, RobotPreview, axis_rotation

ASSET = Path(__file__).resolve().parents[1] / 'assets/go2-preview.json'


@pytest.fixture(scope='module')
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_joint_rotation_and_parent_chain():
    np.testing.assert_allclose(axis_rotation([0, 0, 1], np.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    model = PreviewModel(ASSET)
    before, _ = model.triangles(model.default)
    pose = {**model.default, 'lf_upper_leg_joint': 1.1}
    after, _ = model.triangles(pose)
    offset = 0
    for part in model.parts:
        count = sum(len(mesh['faces']) for mesh in part['meshes'])
        if count:
            moved = not np.allclose(before[offset:offset+count], after[offset:offset+count])
            assert moved == (part['name'] in {'lf_upper_leg_link', 'lf_lower_leg_link', 'lf_foot_link'})
        offset += count
    assert len(before) < 800


def test_static_frames_cached_and_stale_pose_frozen(app):
    rclpy.init()
    widget = RobotPreview(ASSET)
    try:
        widget.show()
        app.processEvents()
        assert widget.timer.interval() == 100
        assert not widget.connected
        frame = widget.frame
        widget.poll()
        app.processEvents()
        assert widget.frame is frame
        msg = JointState()
        msg.name = list(widget.model.default)
        msg.position = list(widget.model.default.values())
        widget.node.receive(msg)
        widget.poll()
        app.processEvents()
        assert widget.connected
        frame = widget.frame
        widget.node.receive(msg)
        widget.poll()
        app.processEvents()
        assert widget.frame is frame
        msg.position[msg.name.index('lf_upper_leg_joint')] += .2
        widget.node.receive(msg)
        widget.poll()
        app.processEvents()
        assert widget.frame is not frame
        pose = widget.positions.copy()
        widget.node.seen = time.monotonic() - 2
        widget.poll()
        app.processEvents()
        assert not widget.connected and widget.positions == pose
        widget.hide()
        assert not widget.timer.isActive()
    finally:
        widget.shutdown()
        widget.close()
        rclpy.shutdown()


def test_invalid_joint_values_ignored(app):
    rclpy.init()
    widget = RobotPreview(ASSET)
    try:
        msg = JointState()
        msg.name = ['lf_upper_leg_joint', 'rf_upper_leg_joint', 'unknown']
        msg.position = [float('nan'), .8, 1.]
        widget.node.receive(msg)
        assert widget.node.positions == {'rf_upper_leg_joint': .8}
        assert widget.node.subscription.qos_profile.depth == 1
    finally:
        widget.shutdown()
        rclpy.shutdown()
