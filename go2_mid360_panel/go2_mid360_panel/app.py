"""Go2 desktop supervisor."""
import os
from pathlib import Path
import re
import queue
import signal
import shlex
import socket
import sys
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from PyQt5 import QtCore, QtGui, QtWidgets as Q
import rclpy
from std_srvs.srv import Trigger
from .processes import Job, launch_args
from .status import StatusNode
from .robot_preview import RobotPreview
from .motion_dashboard import MotionDashboard
from .teleop import conflict_reason
from .rviz_recovery import owned_rviz

MODES = [('simulation', '仿真', '启动仿真与传感器，可配合键盘遥控。'),
         ('mapping', '建图', '启动仿真，使用 FAST-LIO 构建三维点云地图。'),
         ('localization', '定位', '加载已有点云地图进行定位，不启动导航。'),
         ('navigation', '导航', '加载配套地图，启动定位和 Nav2。')]
ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


def section(title, layout_type=Q.QVBoxLayout):
    frame = Q.QFrame(); frame.setObjectName('section')
    outer = Q.QVBoxLayout(frame); outer.setContentsMargins(18, 16, 18, 16)
    heading = Q.QLabel(title); heading.setObjectName('sectionTitle'); heading.setFixedHeight(24)
    outer.addWidget(heading)
    body = Q.QWidget(); outer.addWidget(body)
    layout = layout_type(body); layout.setContentsMargins(0, 10, 0, 0)
    return frame, layout


class Window(Q.QMainWindow):
    def __init__(self, workspace=None):
        super().__init__()
        share = Path(get_package_share_directory('go2_mid360_panel'))
        self.workspace = Path(workspace) if workspace else share.parents[3]
        self.node = StatusNode()
        self.run_job = self.tool_job = self.teleop_job = None
        self.rviz_job = None
        self.rviz_args = None
        self.rviz_restart_pending = False
        self.running_mode = None
        self.last_job = None
        self.was_ready = False
        self.save_future = None
        self.closing = False
        self.setWindowTitle('Go2 · 仿真控制台')
        screen = QtGui.QGuiApplication.primaryScreen()
        available = screen.availableGeometry() if screen else QtCore.QRect(0, 0, 1200, 1080)
        self.resize(min(1080, available.width() - 40), min(960, available.height() - 60))
        root = Q.QWidget()
        scroll = Q.QScrollArea(); scroll.setWidgetResizable(True)
        scroll.setFrameShape(Q.QFrame.NoFrame); scroll.setWidget(root)
        self.setCentralWidget(scroll)
        layout = Q.QVBoxLayout(root); layout.setContentsMargins(20, 16, 20, 14); layout.setSpacing(12)
        header = Q.QFrame(); header.setObjectName('appHeader'); header.setFixedHeight(172)
        header_layout = Q.QHBoxLayout(header)
        header_layout.setContentsMargins(16, 10, 16, 10); header_layout.setSpacing(12)
        portrait = Q.QLabel(); portrait.setFixedSize(140, 132)
        portrait.setPixmap(QtGui.QPixmap(str(share / 'assets/go2.png')).scaled(
            portrait.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        portrait.setAlignment(QtCore.Qt.AlignCenter)
        header_layout.addWidget(portrait)
        identity = Q.QVBoxLayout(); identity.setSpacing(12)
        title = Q.QLabel('Go2 仿真控制台'); title.setObjectName('title')
        identity.addStretch(); identity.addWidget(title)
        self.run_state = Q.QLabel('已停止'); self.run_state.setObjectName('runState')
        self.run_state.setWordWrap(True); self.run_state.setMaximumWidth(220)
        identity.addWidget(self.run_state, 0, QtCore.Qt.AlignLeft); identity.addStretch()
        header_layout.addLayout(identity); header_layout.addStretch()
        self.robot_preview = RobotPreview(share / 'assets/go2-preview.json')
        self.motion_dashboard = MotionDashboard(self.robot_preview)
        header_layout.addWidget(self.motion_dashboard)
        self.setWindowIcon(QtGui.QIcon(str(share / 'assets/go2-panel.svg')))
        layout.addWidget(header)
        upper = Q.QHBoxLayout(); upper.setSpacing(18); layout.addLayout(upper)
        run_box, form = section('运行控制')
        self.mode = Q.QComboBox(self); self.mode.hide()
        for key, name, _ in MODES: self.mode.addItem(name, key)
        self.mode.setCurrentIndex(3)
        modes = Q.QHBoxLayout(); modes.setSpacing(6)
        self.mode_buttons = []; self.mode_group = Q.QButtonGroup(self)
        for index, (_, name, description) in enumerate(MODES):
            button = Q.QPushButton(name); button.setObjectName('mode')
            button.setCheckable(True); button.setChecked(index == 3)
            button.setToolTip(description)
            self.mode_group.addButton(button, index)
            modes.addWidget(button); self.mode_buttons.append(button)
        self.mode_group.idClicked.connect(self.mode.setCurrentIndex)
        self.mode.currentIndexChanged.connect(lambda index: self.mode_buttons[index].setChecked(True))
        form.addLayout(modes)
        self.description = Q.QLabel(); self.description.setWordWrap(True); form.addWidget(self.description)
        self.mode.currentIndexChanged.connect(self.describe); self.describe()
        options = Q.QHBoxLayout(); self.gui = Q.QCheckBox('Gazebo 窗口'); self.rviz = Q.QCheckBox('RViz')
        self.gui.setChecked(True); self.rviz.setChecked(True)
        self.terminal = Q.QCheckBox('日志终端'); self.terminal.setChecked(True)
        self.terminal.setToolTip('启动时打开实时日志终端，Ctrl+C 停止任务。关闭日志窗口不停止任务。')
        options.addWidget(self.gui); options.addWidget(self.rviz); options.addWidget(self.terminal)
        form.addLayout(options)
        buttons = Q.QHBoxLayout(); self.start = Q.QPushButton('启动导航'); self.start.setObjectName('primary')
        self.stop = Q.QPushButton('停止任务'); self.stop.setObjectName('stop'); self.stop.setEnabled(False)
        buttons.addWidget(self.start); buttons.addWidget(self.stop); form.addLayout(buttons)
        self.start.clicked.connect(self.start_run); self.stop.clicked.connect(self.stop_run)
        teleop_row = Q.QHBoxLayout()
        self.teleop_button = Q.QPushButton('键盘遥控')
        self.teleop_stop = Q.QPushButton('结束遥控')
        self.teleop_button.clicked.connect(self.start_teleop)
        self.teleop_stop.clicked.connect(self.stop_teleop)
        self.teleop_stop.setEnabled(False)
        teleop_row.addWidget(self.teleop_button); teleop_row.addWidget(self.teleop_stop)
        self.restart_rviz_button = Q.QPushButton('重启 RViz')
        self.restart_rviz_button.setToolTip('仅重开本次任务的 RViz，不停止仿真、定位或导航。')
        self.restart_rviz_button.setEnabled(False)
        self.restart_rviz_button.clicked.connect(self.restart_rviz)
        teleop_row.addWidget(self.restart_rviz_button)
        form.addLayout(teleop_row)
        self.teleop_note = Q.QLabel('仿真就绪后可用；导航运行时禁用。')
        self.teleop_note.setWordWrap(True); self.teleop_note.setObjectName('subtitle')
        form.addWidget(self.teleop_note)
        form.addStretch()
        upper.addWidget(run_box, 1)
        status_box, grid = section('系统状态', Q.QGridLayout); self.indicators = {}
        for row, name in enumerate(['仿真时钟', '雷达', 'FAST-LIO', '定位', 'TF', 'Nav2']):
            cell = Q.QFrame(); cell.setObjectName('statusCell')
            cell_layout = Q.QVBoxLayout(cell); cell_layout.setContentsMargins(12, 9, 12, 9)
            caption = Q.QLabel(name); caption.setObjectName('subtitle')
            label = Q.QLabel('未检测'); cell_layout.addWidget(caption); cell_layout.addWidget(label)
            grid.addWidget(cell, row // 3, row % 3); self.indicators[name] = label
        status_box.setToolTip('检测当前 ROS 通信域中的数据、TF 和 Nav2 激活状态。')
        upper.addWidget(status_box, 1)
        map_box, maps = section('地图目录'); layout.addWidget(map_box)
        row = Q.QHBoxLayout()
        self.folder = Q.QLineEdit(str(self.workspace / 'maps/go2_workshop')); row.addWidget(self.folder)
        self.folder.setToolTip('导航读取地图、建图保存和二维转换均使用此目录。')
        self.browse = Q.QPushButton('选择…'); self.browse.clicked.connect(self.choose_folder)
        row.addWidget(self.browse); maps.addLayout(row)
        self.map_info = Q.QLabel(); self.folder.textChanged.connect(self.update_map_info)
        maps.addWidget(self.map_info); self.update_map_info()
        self.map_usage = Q.QLabel(); self.map_usage.setObjectName('subtitle')
        self.map_usage.setWordWrap(True); maps.addWidget(self.map_usage)

        operations = Q.QHBoxLayout(); operations.setSpacing(18)
        layout.addLayout(operations)
        save_box, save_layout = section('建图保存')
        self.save_note = Q.QLabel('仅建图模式运行且保存服务就绪时可用。')
        self.save_note.setWordWrap(True); self.save_note.setMinimumHeight(36); save_layout.addWidget(self.save_note)
        save_output = Q.QLabel('输出：map.pcd'); save_output.setObjectName('subtitle')
        save_layout.addWidget(save_output)
        self.save_button = Q.QPushButton('保存三维点云')
        self.save_button.clicked.connect(self.save_map)
        self.save_button.setFixedWidth(144)
        save_actions = Q.QHBoxLayout(); save_actions.addWidget(self.save_button); save_actions.addStretch()
        save_layout.addLayout(save_actions)
        operations.addWidget(save_box, 1)

        convert_box, convert_layout = section('二维地图转换')
        self.convert_note = Q.QLabel('停止任务后，将点云转换为二维导航地图。')
        self.convert_note.setWordWrap(True); self.convert_note.setMinimumHeight(36); convert_layout.addWidget(self.convert_note)
        output_note = Q.QLabel('输出：map.yaml + map.pgm'); output_note.setObjectName('subtitle')
        convert_layout.addWidget(output_note)
        convert_actions = Q.QHBoxLayout()
        self.convert_button = Q.QPushButton('转换二维地图')
        self.convert_button.clicked.connect(self.convert_map)
        self.convert_button.setFixedWidth(144)
        convert_actions.addWidget(self.convert_button); convert_actions.addStretch()
        self.advanced_toggle = Q.QToolButton()
        self.advanced_toggle.setText('高级设置'); self.advanced_toggle.setCheckable(True)
        self.advanced_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.advanced_toggle.setArrowType(QtCore.Qt.RightArrow)
        self.advanced_toggle.setFixedHeight(self.convert_button.sizeHint().height())
        convert_actions.addWidget(self.advanced_toggle)
        convert_layout.addLayout(convert_actions)
        self.advanced_settings, params = section('二维地图转换 · 高级设置', Q.QGridLayout)
        params.setHorizontalSpacing(18)
        self.zmin = self.spin(-10, 10, .35, .05)
        self.zmax = self.spin(-10, 10, 2.6, .05)
        self.resolution = self.spin(.01, 1, .05, .01)
        for column, (name, widget) in enumerate([('投影高度下限 (m)', self.zmin),
                                                 ('投影高度上限 (m)', self.zmax),
                                                 ('栅格分辨率 (m)', self.resolution)]):
            params.addWidget(Q.QLabel(name), 0, column)
            params.addWidget(widget, 1, column)
            params.setColumnStretch(column, 1)
            widget.valueChanged.connect(self.projection_changed)
        self.reset_projection = Q.QPushButton('恢复默认参数')
        self.reset_projection.clicked.connect(self.reset_projection_defaults)
        params.addWidget(self.reset_projection, 2, 2, QtCore.Qt.AlignRight)
        layout.addWidget(self.advanced_settings)
        self.advanced_settings.hide()
        self.advanced_toggle.toggled.connect(self.toggle_advanced)
        operations.addWidget(convert_box, 1)
        self.map_message = Q.QLabel(''); self.map_message.setWordWrap(True)
        layout.addWidget(self.map_message); self.map_message.hide()
        log_header = Q.QHBoxLayout()
        log_title = Q.QLabel('运行日志'); log_title.setObjectName('sectionTitle')
        log_header.addWidget(log_title); log_header.addStretch()
        self.open_terminal_button = Q.QPushButton('打开日志终端')
        self.open_terminal_button.clicked.connect(self.open_log_terminal)
        self.open_terminal_button.setEnabled(False); log_header.addWidget(self.open_terminal_button)
        export = Q.QPushButton('导出日志'); export.clicked.connect(self.export_log); log_header.addWidget(export)
        clear = Q.QPushButton('清空显示'); clear.clicked.connect(lambda: self.logs.clear()); log_header.addWidget(clear); layout.addLayout(log_header)
        self.logs = Q.QPlainTextEdit(); self.logs.setReadOnly(True); self.logs.document().setMaximumBlockCount(2500); self.logs.setMinimumHeight(96); self.logs.setSizePolicy(Q.QSizePolicy.Expanding, Q.QSizePolicy.Ignored); layout.addWidget(self.logs, 1)
        self.statusBar().showMessage(f'工作空间：{self.workspace}    ROS_DOMAIN_ID：{os.environ.get("ROS_DOMAIN_ID", "0")}')
        self.describe()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(200)

    @staticmethod
    def spin(lo, hi, value, step):
        w = Q.QDoubleSpinBox(); w.setRange(lo, hi); w.setDecimals(2); w.setSingleStep(step); w.setValue(value); return w

    def toggle_advanced(self, expanded):
        self.advanced_settings.setVisible(expanded)
        self.advanced_toggle.setArrowType(QtCore.Qt.DownArrow if expanded else QtCore.Qt.RightArrow)

    def reset_projection_defaults(self):
        self.zmin.setValue(.35)
        self.zmax.setValue(2.6)
        self.resolution.setValue(.05)

    def projection_changed(self):
        custom = (self.zmin.value(), self.zmax.value(), self.resolution.value()) != (.35, 2.6, .05)
        self.advanced_toggle.setText('高级设置 · 已修改' if custom else '高级设置')

    def restart_rviz(self):
        if not self.run_job or self.run_job.deadline is not None or not self.rviz_args or self.closing:
            return
        if self.rviz_job:
            self.rviz_restart_pending = True
            self.rviz_job.stop()
            return
        try:
            self.rviz_restart_pending = False
            self.rviz_job = Job([sys.executable, '-m', 'go2_mid360_panel.rviz_recovery',
                                 str(self.run_job.process.pid), *self.rviz_args], self.workspace)
        except OSError as exc:
            self.error(str(exc))

    def start_teleop(self):
        if self.teleop_job or self.closing:
            return
        reason = conflict_reason(self.node)
        if self.running_mode == 'navigation' or reason:
            self.error(reason or '请先停止导航，再以仿真、建图或定位模式启动。')
            return
        if not self.node.values()['仿真时钟']:
            self.error('请先启动仿真，等待仿真时钟就绪。')
            return
        try:
            self.teleop_job = Job([sys.executable, '-m', 'go2_mid360_panel.teleop'], self.workspace)
            self.log('正在启动键盘遥控。')
        except OSError as exc:
            self.error(str(exc))

    def stop_teleop(self):
        if self.teleop_job:
            self.teleop_job.stop()

    def open_log_terminal(self):
        if self.last_job:
            self.last_job.open_terminal()

    def describe(self):
        self.description.setText(MODES[self.mode.currentIndex()][2])
        if hasattr(self, 'start'):
            self.start.setText('启动' + self.mode.currentText())
        if hasattr(self, 'map_usage'):
            self.map_usage.setText({
                'simulation': '当前仿真模式不读取地图；此目录仍可用于二维地图转换。',
                'mapping': '建图输出目录。建议为新地图选择独立目录，避免覆盖已有地图。',
                'localization': '定位读取此目录中的 map.pcd。',
                'navigation': '导航读取此目录中的 map.pcd、map.yaml 及其引用的栅格图。',
            }[self.mode.currentData()])

    def directory(self): return Path(self.folder.text().strip()).expanduser().resolve()

    def choose_folder(self):
        path = Q.QFileDialog.getExistingDirectory(self, '选择地图目录', self.folder.text())
        if path: self.folder.setText(path)

    def update_map_info(self):
        folder = self.directory()
        self.map_info.setText('    '.join(f'{name}：' + ('已有' if (folder/name).is_file() else '未生成') for name in ['map.pcd', 'map.yaml', 'map.pgm']))

    def log(self, text): self.logs.appendPlainText(ANSI.sub('', text))

    def error(self, text):
        self.log('错误：' + text); Q.QMessageBox.warning(self, '无法执行', text)

    def confirm(self, text):
        return Q.QMessageBox.question(self, '确认覆盖', text, Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No) == Q.QMessageBox.Yes

    def start_run(self):
        if self.run_job or self.tool_job or self.closing: return
        mode = self.mode.currentData()
        if self.teleop_job:
            self.error('请先结束键盘遥控，再启动新任务。'); return
        if not self.folder.text().strip(): self.error('请填写地图目录。'); return
        folder = self.directory()
        try:
            with socket.socket() as sock:
                sock.settimeout(.15)
                if sock.connect_ex(('127.0.0.1', 11345)) == 0:
                    self.error('Gazebo 已在运行。请先结束原启动进程，再从此窗口启动。'); return
            required = ['map.pcd'] if mode == 'localization' else ['map.pcd', 'map.yaml'] if mode == 'navigation' else []
            missing = [str(folder/n) for n in required if not (folder/n).is_file()]
            if missing: self.error('缺少地图文件：\n' + '\n'.join(missing)); return
            if mode == 'navigation':
                try:
                    metadata = yaml.safe_load((folder / 'map.yaml').read_text())
                    image_name = metadata.get('image') if isinstance(metadata, dict) else None
                    if not isinstance(image_name, str) or not image_name.strip():
                        self.error('map.yaml 中未指定有效的 image 文件。'); return
                    if not (folder / image_name).is_file():
                        self.error('map.yaml 引用的栅格图不存在：' + str(folder / image_name)); return
                except (yaml.YAMLError, UnicodeError) as exc:
                    self.error('map.yaml 格式错误：' + str(exc)); return
            if mode == 'mapping':
                if (folder/'map.pcd').exists() and not self.confirm(f'建图保存将覆盖：\n{folder / "map.pcd"}\n建议为新地图选择独立目录。'): return
                folder.mkdir(parents=True, exist_ok=True)
            args = launch_args(mode, folder, self.gui.isChecked(), self.rviz.isChecked())
            # A new Gazebo session resets simulation time; discard old TF and service state.
            self.node.listener.unregister()
            self.node.destroy_node()
            self.node = StatusNode()
            self.save_future = None
            self.log('$ ' + shlex.join(args))
            self.run_job = Job(args, self.workspace, show_terminal=self.terminal.isChecked())
            self.rviz_args = None
            self.rviz_restart_pending = False
            self.last_job = self.run_job; self.was_ready = False; self.running_mode = mode
            self.run_state.setText('正在启动 · ' + self.mode.currentText()); self.set_locked(True)
            self.map_message.clear()
        except OSError as exc: self.error(str(exc))

    def set_locked(self, locked):
        for widget in [self.start, self.mode, self.gui, self.rviz, self.terminal, self.folder, self.browse, self.advanced_settings, *self.mode_buttons]: widget.setEnabled(not locked)
        self.stop.setEnabled(locked and self.run_job is not None)

    def stop_run(self):
        self.rviz_restart_pending = False
        if self.rviz_job:
            self.rviz_job.stop()
        self.stop_teleop()
        if self.run_job:
            self.run_job.stop(); self.stop.setEnabled(False); self.run_state.setText('正在停止任务')

    def save_map(self):
        if not self.run_job or self.running_mode != 'mapping' or self.save_future: return
        if not self.node.save.service_is_ready(): self.error('地图保存服务尚未就绪。'); return
        self.save_future = self.node.save.call_async(Trigger.Request()); self.save_started = time.monotonic()
        self.map_message.setText('正在保存点云…')

    def convert_map(self):
        if self.run_job or self.tool_job: return
        if not self.folder.text().strip(): self.error('请填写地图目录。'); return
        folder = self.directory()
        if not (folder/'map.pcd').is_file(): self.error('地图目录中没有 map.pcd。'); return
        if self.zmin.value() >= self.zmax.value(): self.error('高度下限必须小于上限。'); return
        if any((folder/n).exists() for n in ['map.yaml', 'map.pgm']) and not self.confirm(f'将覆盖目录中的 map.yaml 和 map.pgm，包括手工编辑结果：\n{folder}'): return
        args = ['ros2', 'run', 'go2_fastlio_localization', 'pcd_to_nav2_map', str(folder/'map.pcd'), str(folder/'map'),
                '--resolution', str(self.resolution.value()), '--z-min', str(self.zmin.value()), '--z-max', str(self.zmax.value()),
                '--padding', '.30', '--min-points', '2', '--dilation-cells', '1']
        try:
            self.tool_job = Job(args, self.workspace, show_terminal=self.terminal.isChecked())
            self.last_job = self.tool_job; self.log('$ ' + shlex.join(args)); self.set_locked(True)
            self.run_state.setText('正在转换地图')
            self.map_message.setText('正在转换二维地图…')
        except OSError as exc: self.error(str(exc))

    def export_log(self):
        path, _ = Q.QFileDialog.getSaveFileName(self, '导出日志', str(self.workspace/'panel.log'), '文本日志 (*.log)')
        if path:
            try: Path(path).write_text(self.logs.toPlainText() + '\n')
            except OSError as exc: self.error(str(exc))

    def tick(self):
        for _ in range(8): rclpy.spin_once(self.node, timeout_sec=0)
        states = self.node.values()
        mode = self.running_mode or self.mode.currentData()
        expected = ['仿真时钟', '雷达']
        if mode != 'simulation': expected.append('FAST-LIO')
        if mode in ('localization', 'navigation'): expected += ['定位', 'TF']
        if mode == 'navigation': expected.append('Nav2')
        for name, active in states.items():
            if name not in expected:
                text, color = '本模式不使用', '#8193a4'
            elif self.run_job:
                text, color = ('已就绪', '#5ed7ab') if active else ('等待数据', '#d1b57b')
            else:
                text, color = ('外部已就绪', '#93bacb') if active else ('未启动', '#93a8ba')
            self.indicators[name].setText(text)
            self.indicators[name].setStyleSheet('color: ' + color + ';')
        if self.run_job:
            if self.run_job.deadline is not None:
                self.run_state.setText('正在停止任务')
                self.stop.setEnabled(False)
            else:
                ready = all(states[name] for name in expected)
                label = '已就绪' if ready else '数据中断' if self.was_ready else '正在启动'
                self.run_state.setText(label + ' · ' + self.mode.currentText())
                self.was_ready |= ready
        if self.run_job and self.rviz_args is None:
            matches = owned_rviz(self.run_job.process.pid)
            if matches:
                self.rviz_args = matches[0][2]
        for attr in ['run_job', 'tool_job', 'teleop_job', 'rviz_job']:
            job = getattr(self, attr)
            if job is None: continue
            active = job.tick()
            for _ in range(180):
                try: self.log(job.lines.get_nowait())
                except queue.Empty: break
            if not active:
                code = job.process.poll(); stopped = job.stop_requested
                self.log(f'进程结束，退出码：{code}')
                setattr(self, attr, None)
                if attr == 'rviz_job':
                    if self.rviz_restart_pending:
                        self.restart_rviz()
                    continue
                if attr == 'teleop_job':
                    self.log('键盘遥控已结束。')
                    continue
                if attr == 'run_job':
                    self.stop_teleop()
                    if self.rviz_job: self.rviz_job.stop()
                    self.running_mode = None; self.run_state.setText('已停止' if stopped or code == 0 else f'启动进程异常退出（{code}），请查看日志')
                else:
                    self.run_state.setText('已停止')
                    self.map_message.setText('转换完成，可启动导航加载新地图。' if code == 0 else '转换未完成，请查看日志。')
                self.set_locked(False); self.update_map_info()
        if self.save_future:
            if self.save_future.done():
                try:
                    result = self.save_future.result(); text = ('保存成功：' if result.success else '保存失败：') + result.message
                except Exception as exc: text = '保存失败：' + str(exc)
                self.map_message.setText(text); self.log(text); self.save_future = None; self.update_map_info()
            elif time.monotonic()-self.save_started > 30:
                self.node.save.remove_pending_request(self.save_future); self.save_future = None
                self.map_message.setText('保存响应超时，请检查日志和文件；操作可能仍在执行。')
        self.map_message.setVisible(bool(self.map_message.text()))
        busy = bool(self.run_job or self.tool_job)
        has_pcd = bool(self.folder.text().strip()) and (self.directory() / 'map.pcd').is_file()
        self.convert_button.setEnabled(not busy and not self.closing and has_pcd)
        self.convert_note.setText('请先停止当前任务，再转换二维地图。' if busy else
                                  '所选目录缺少 map.pcd，请先建图并保存。' if not has_pcd else
                                  '停止任务后，将点云转换为二维导航地图。')
        can_save = bool(self.run_job and self.running_mode == 'mapping' and self.run_job.deadline is None and not self.save_future and self.node.save.service_is_ready())
        self.save_button.setEnabled(can_save)
        self.save_note.setText('正在保存三维点云，请稍候。' if self.save_future else
                               '仅建图模式运行时可保存三维点云。' if not self.run_job or self.running_mode != 'mapping' else
                               '正在停止任务，等待点云保存。' if self.run_job.deadline is not None else
                               '保存服务就绪，可手动保存当前点云。' if can_save else
                               '正在启动建图，等待保存服务就绪。')
        terminal_open = bool(self.last_job and self.last_job.terminal_process is not None and self.last_job.terminal_process.poll() is None)
        self.open_terminal_button.setEnabled(self.last_job is not None and not terminal_open)
        self.open_terminal_button.setText('日志终端已打开' if terminal_open else '打开日志终端')
        self.open_terminal_button.setToolTip('请从任务栏切换到日志终端。' if terminal_open else '打开最近任务的完整日志。')

        reason = conflict_reason(self.node)
        if self.running_mode == 'navigation':
            reason = '导航运行中，键盘遥控已禁用。'
        if self.teleop_job:
            # The session monitor stops the keyboard if an external navigation task starts.
            if self.running_mode == 'navigation':
                self.stop_teleop()
            self.teleop_note.setText('W/X 前后，A/D 转向，S/空格停止；R/F 调速，Ctrl+C 退出。')
        else:
            self.teleop_note.setText(reason or ('点击打开遥控终端；按住方向键行走，S/空格停止。' if states['仿真时钟'] else '请先启动仿真，等待仿真时钟就绪。'))
        self.teleop_button.setEnabled(not self.teleop_job and not reason and states['仿真时钟'] and not self.closing and not self.tool_job and not (self.run_job and self.run_job.deadline))
        self.teleop_stop.setEnabled(bool(self.teleop_job and self.teleop_job.deadline is None))
        if self.teleop_job:
            self.start.setEnabled(False)
        elif not busy and not self.closing:
            self.start.setEnabled(True)
        self.restart_rviz_button.setEnabled(bool(self.run_job and self.rviz_args and self.run_job.deadline is None and not self.closing and not self.rviz_restart_pending))
        if self.closing and not busy and not self.teleop_job and not self.rviz_job: self.close()

    def closeEvent(self, event):
        if self.run_job or self.tool_job or self.teleop_job or self.rviz_job:
            self.closing = True; self.setEnabled(False); self.stop_run()
            if self.tool_job: self.tool_job.stop()
            event.ignore(); return
        self.timer.stop(); self.robot_preview.shutdown(); self.node.destroy_node(); event.accept()


def configure_application(app):
    app.setStyle('Fusion'); app.setFont(QtGui.QFont('Noto Sans CJK SC', 10))
    palette = QtGui.QPalette()
    for role, color in [(QtGui.QPalette.Window, '#101820'),
                        (QtGui.QPalette.WindowText, '#dce6ef'),
                        (QtGui.QPalette.Base, '#101e2b'),
                        (QtGui.QPalette.Text, '#dce6ef'),
                        (QtGui.QPalette.Button, '#223547'),
                        (QtGui.QPalette.ButtonText, '#deebf5'),
                        (QtGui.QPalette.Highlight, '#087fae'),
                        (QtGui.QPalette.HighlightedText, '#ffffff')]:
        palette.setColor(role, QtGui.QColor(color))
    app.setPalette(palette)
    assets = Path(get_package_share_directory('go2_mid360_panel')) / 'assets'
    app.setStyleSheet((assets / 'panel.qss').read_text().replace('@ASSETS@', str(assets)))
    app.setWindowIcon(QtGui.QIcon(str(assets / 'go2-panel.svg')))
    app.setApplicationName('Go2 Control')
    app.setDesktopFileName('go2-mid360-panel')



def main():
    rclpy.init()
    app = Q.QApplication(sys.argv); configure_application(app)
    window = Window(); window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    try: return app.exec_()
    finally:
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
