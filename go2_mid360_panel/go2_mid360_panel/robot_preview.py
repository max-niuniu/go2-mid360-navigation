"""Small orthographic robot preview driven by read-only joint states."""
import json
import math
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from .motion_dashboard import MotionState


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


class PreviewModel:
    def __init__(self, path):
        self.parts = json.loads(path.read_text())
        self.default = {}
        for part in self.parts:
            part['origin'] = np.array(part['origin'])
            name = part['joint']
            if name:
                self.default[name] = .75 if 'upper' in name else -1.5 if 'lower' in name else 0.
            for mesh in part['meshes']:
                mesh['vertices'] = np.array(mesh['vertices'])
                mesh['faces'] = np.array(mesh['faces'], dtype=int)
        camera = np.array([1.1, -1.8, .9]); camera /= np.linalg.norm(camera)
        right = np.cross([0, 0, 1], camera); right /= np.linalg.norm(right)
        up = np.cross(camera, right)
        self.camera = np.array([right, up, camera])
        triangles, _ = self.triangles(self.default)
        projected = triangles @ self.camera.T
        low, high = projected[:, :, :2].min(axis=(0, 1)), projected[:, :, :2].max(axis=(0, 1))
        self.center = (low + high) / 2
        self.extent = (high - low) * 1.15

    def triangles(self, positions):
        transforms = {}
        triangles, colors = [], []
        for part in self.parts:
            local = part['origin'].copy()
            if part['joint']:
                local[:3, :3] = local[:3, :3] @ axis_rotation(
                    part['axis'], positions.get(part['joint'], self.default[part['joint']]))
            world = transforms.get(part['parent'], np.eye(4)) @ local
            transforms[part['name']] = world
            for mesh in part['meshes']:
                vertices = mesh['vertices'] @ world[:3, :3].T + world[:3, 3]
                triangles.append(vertices[mesh['faces']])
                colors.extend([mesh['color']] * len(mesh['faces']))
        return np.concatenate(triangles), np.array(colors)

    def render(self, positions, size, connected=False, ratio=1.):
        image = QtGui.QImage(round(size.width() * ratio), round(size.height() * ratio),
                             QtGui.QImage.Format_ARGB32_Premultiplied)
        image.setDevicePixelRatio(ratio)
        image.fill(QtCore.Qt.transparent)
        triangles, colors = self.triangles(positions)
        normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
        visible = normal @ self.camera[2] > 0
        triangles, colors, normal = triangles[visible], colors[visible], normal[visible]
        light = np.array([.3, -.4, 1.]); light /= np.linalg.norm(light)
        colors = np.clip(colors * (.52 + .48 * np.maximum(normal @ light, 0))[:, None], 0, 255)
        projected = triangles @ self.camera.T
        scale = min((size.width() - 8) / self.extent[0], (size.height() - 8) / self.extent[1])
        points = (projected[:, :, :2] - self.center) * [scale, -scale]
        points += [size.width() / 2, size.height() / 2]
        painter = QtGui.QPainter(image)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setOpacity(1. if connected else .65)
        for i in np.argsort(projected[:, :, 2].mean(axis=1)):
            color = QtGui.QColor(*(int(c) for c in colors[i]))
            painter.setPen(QtGui.QPen(color, .35))
            painter.setBrush(color)
            painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(float(x), float(y)) for x, y in points[i]]))
        painter.end()
        return image


class JointPreviewNode(Node):
    def __init__(self, names):
        super().__init__('go2_joint_preview')
        self.names = set(names)
        self.positions = {}
        self.seen = 0.
        self.motion = MotionState()
        self.odometry = self.create_subscription(
            Odometry, '/odom/ground_truth', self.motion.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def receive(self, msg):
        values = {name: value for name, value in zip(msg.name, msg.position)
                  if name in self.names and math.isfinite(value)}
        if values:
            self.positions.update(values)
            self.seen = time.monotonic()


class RobotPreview(QtWidgets.QWidget):
    motion_changed = QtCore.pyqtSignal(object)

    def __init__(self, asset, parent=None):
        super().__init__(parent)
        self.setFixedSize(156, 100)
        self.setAccessibleName('Go2 三维姿态预览')
        self.model = PreviewModel(asset)
        self.node = JointPreviewNode(self.model.default)
        self.positions = self.model.default.copy()
        self.connected = False
        self.frame = None
        self.setToolTip('三维预览 · 未连接')
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.poll)

    def poll(self):
        if not self.isVisible() or self.window().isMinimized() or not rclpy.ok():
            return
        # Bound callback work; both subscriptions discard queued old samples.
        for _ in range(2):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.motion_changed.emit(self.node.motion.snapshot())
        connected = (time.monotonic() - self.node.seen < 1.5
                     and self.node.names <= self.node.positions.keys())
        positions = self.node.positions if connected else self.positions
        changed = any(abs(positions.get(k, v) - v) > .012 for k, v in self.positions.items())
        if connected != self.connected or changed:
            self.positions = {k: positions.get(k, v) for k, v in self.positions.items()}
            self.connected = connected
            self.frame = None
            self.setToolTip('三维预览 · 关节实时同步' if connected else '三维预览 · 未连接')
            self.update()

    def paintEvent(self, event):
        ratio = self.devicePixelRatioF()
        if self.frame is None or self.frame.devicePixelRatio() != ratio:
            self.frame = self.model.render(self.positions, self.size(), self.connected, ratio)
        painter = QtGui.QPainter(self)
        painter.drawImage(0, 0, self.frame)

    def showEvent(self, event):
        self.timer.start()
        super().showEvent(event)

    def hideEvent(self, event):
        self.timer.stop()
        super().hideEvent(event)

    def shutdown(self):
        self.timer.stop()
        self.node.destroy_node()
