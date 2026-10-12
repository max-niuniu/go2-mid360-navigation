"""Read-only Gazebo motion feedback and compact instrument panel."""
import math
import time

from PyQt5 import QtCore, QtGui, QtWidgets


def motion_label(vx, vy, wz):
    if math.hypot(vx, vy) < .035:
        return '原地左旋' if wz >= .08 else '原地右旋' if wz <= -.08 else '静止'
    if abs(vy) > abs(vx):
        direction = '左移' if vy > 0 else '右移'
    else:
        direction = '前进' if vx >= 0 else '后退'
    return direction + (' · 左转' if wz >= .08 else ' · 右转' if wz <= -.08 else '')


class MotionState:
    def __init__(self):
        self.seen = 0.
        self.stamp = None
        self.velocity = (0., 0., 0.)

    def receive(self, msg):
        # gazebo_ros_p3d publishes world-frame twist despite its body child_frame_id.
        if msg.header.frame_id.lstrip('/') != 'world':
            return
        q = msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = (q.x, q.y, q.z, q.w, v.x, v.y, w.z)
        if not all(math.isfinite(value) for value in values):
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            return
        x, y, z, qw = (value / norm for value in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2*(qw*z + x*y), 1 - 2*(y*y + z*z))
        c, s = math.cos(yaw), math.sin(yaw)
        velocity = (c*v.x + s*v.y, -s*v.x + c*v.y, w.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = time.monotonic()
        if stamp == self.stamp:
            return
        dt = stamp - self.stamp if self.stamp is not None else 0.
        # A short display-only filter suppresses gait vibration and sensor noise.
        alpha = 1. - math.exp(-dt / .18) if 0 < dt < 1 and now-self.seen < 1.5 else 1.
        self.velocity = tuple(old + alpha*(new-old) for old, new in zip(self.velocity, velocity))
        self.stamp, self.seen = stamp, now

    def snapshot(self):
        if self.stamp is None or time.monotonic() - self.seen >= 1.5:
            return None
        vx, vy, wz = self.velocity
        if math.hypot(vx, vy) < .035:
            vx = vy = 0.
        if abs(wz) < .08:
            wz = 0.
        return (vx, vy, wz)


class MotionGauge(QtWidgets.QWidget):
    def __init__(self, caption, unit, signed=False):
        super().__init__()
        self.caption, self.unit, self.signed = caption, unit, signed
        self.value = None
        self.setFixedSize(126, 112)
        self.setAccessibleName(caption)
        self.setToolTip('水平实际速度；低于 0.035 m/s 的抖动按零显示。' if not signed else
                        '绕竖直轴的实际角速度；正值左转，负值右转。低于 0.08 rad/s 的抖动按零显示。')

    def set_value(self, value):
        value = None if value is None else round(value, 2)
        if value != self.value:
            self.value = value
            self.setAccessibleDescription('未连接' if value is None else f'{value:.2f} {self.unit}')
            self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self); p.setRenderHint(QtGui.QPainter.Antialiasing)
        bounds = QtCore.QRectF(20, 25, 86, 86)
        base = 2. if self.signed else 1.
        limit = max(base, math.ceil(abs(self.value or 0.) / base) * base)
        p.setPen(QtGui.QPen(QtGui.QColor('#2d485e'), 4, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        p.drawArc(bounds, 210*16, -240*16)
        if self.value is not None:
            color = '#50cfef' if not self.signed else '#7bafea'
            p.setPen(QtGui.QPen(QtGui.QColor(color), 4, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
            start = 90 if self.signed else 210
            sweep = self.value/limit * (120 if self.signed else -240)
            p.drawArc(bounds, start*16, round(sweep*16))
        p.setPen(QtGui.QColor('#9fb5c8')); font = self.font(); font.setPixelSize(12); p.setFont(font)
        p.drawText(QtCore.QRectF(0, 0, 126, 18), QtCore.Qt.AlignCenter, self.caption)
        p.drawText(QtCore.QRectF(0, 74, 126, 18), QtCore.Qt.AlignCenter, self.unit)
        font.setPixelSize(10); p.setFont(font)
        p.drawText(QtCore.QRectF(0, 97, 54, 14), QtCore.Qt.AlignCenter, f'左 +{limit:g}' if self.signed else '0')
        p.drawText(QtCore.QRectF(72, 97, 54, 14), QtCore.Qt.AlignCenter, f'右 −{limit:g}' if self.signed else f'{limit:g}')
        font.setFamily('DejaVu Sans'); font.setPixelSize(23); font.setBold(True); p.setFont(font)
        p.setPen(QtGui.QColor('#eff8ff' if self.value is not None else '#8299ab'))
        text = '—' if self.value is None else (f'{self.value:+.2f}' if self.signed and self.value else f'{self.value:.2f}')
        p.drawText(QtCore.QRectF(0, 47, 126, 30), QtCore.Qt.AlignCenter, text)
        p.end()


class MotionDashboard(QtWidgets.QFrame):
    def __init__(self, preview):
        super().__init__()
        self.setObjectName('motionDashboard'); self.setFixedWidth(444)
        self.setToolTip('Gazebo 实际运动反馈 · /odom/ground_truth。只读显示，不发送控制指令。')
        outer = QtWidgets.QVBoxLayout(self); outer.setContentsMargins(12, 8, 12, 6); outer.setSpacing(0)
        heading = QtWidgets.QHBoxLayout()
        caption = QtWidgets.QLabel('运动状态'); caption.setObjectName('subtitle')
        self.status = QtWidgets.QLabel('●  未连接'); self.status.setObjectName('motionState')
        heading.addWidget(caption); heading.addStretch(); heading.addWidget(self.status)
        outer.addLayout(heading)
        instruments = QtWidgets.QHBoxLayout(); instruments.setSpacing(0)
        instruments.addWidget(preview, 0, QtCore.Qt.AlignVCenter)
        self.speed = MotionGauge('线速度', 'm/s')
        self.turn = MotionGauge('角速度', 'rad/s', signed=True)
        instruments.addWidget(self.speed); instruments.addWidget(self.turn)
        outer.addLayout(instruments)
        self.previous = None
        preview.motion_changed.connect(self.update_motion)

    def update_motion(self, velocity):
        key = None if velocity is None else tuple(round(v, 2) for v in velocity)
        if key == self.previous:
            return
        self.previous = key
        label = '未连接' if velocity is None else motion_label(*velocity)
        self.status.setText('●  ' + label)
        self.status.setStyleSheet('color: #8299ab;' if velocity is None else 'color: #60d5ed;')
        self.speed.set_value(None if velocity is None else math.hypot(*velocity[:2]))
        self.turn.set_value(None if velocity is None else velocity[2])
