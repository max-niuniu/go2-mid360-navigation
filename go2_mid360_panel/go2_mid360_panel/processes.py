"""Own and stop complete launch process groups without touching other sessions."""
import os
from pathlib import Path
import queue
import signal
import shlex
import shutil
import sys
import tempfile
import subprocess
import threading
import time


class Job:
    def __init__(self, argv, cwd, show_terminal=False):
        root = Path(os.environ.get('ROS_LOG_DIR', str(Path.home() / '.ros/log'))) / 'go2_panel'
        root.mkdir(parents=True, exist_ok=True)
        self.log_dir = Path(tempfile.mkdtemp(prefix='task-', dir=root))
        self.log_path = self.log_dir / 'output.log'
        self.log_path.write_text('$ ' + shlex.join(argv) + '\n')
        self.terminal_process = None
        self.terminal_checked = False
        self.terminal_started = 0
        self.stop_requested = False
        self.lines = queue.Queue(maxsize=3000)
        self.process = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True,
                                        errors='replace', start_new_session=True,
                                        env={**os.environ, 'PYTHONUNBUFFERED': '1',
                                             'RCUTILS_COLORIZED_OUTPUT': '0'})
        self.deadline = None
        self.stage = 0
        threading.Thread(target=self._read, daemon=True).start()
        if show_terminal:
            self.open_terminal()

    def open_terminal(self):
        if self.terminal_process is not None and self.terminal_process.poll() is None:
            return
        command = [sys.executable, str(Path(__file__).with_name('terminal.py')), str(self.log_dir)]
        if shutil.which('gnome-terminal'):
            command = ['gnome-terminal', '--wait', '--title=Go2 · 运行日志', '--', *command]
        elif shutil.which('xterm'):
            command = ['xterm', '-T', 'Go2 · 运行日志', '-e', *command]
        else:
            self.lines.put_nowait('无法打开日志终端：请安装 gnome-terminal。任务输出仍显示在控制台。')
            return
        try:
            (self.log_dir / 'terminal-opened').unlink(missing_ok=True)
            with (self.log_dir / 'terminal-error.log').open('a') as errors:
                self.terminal_process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                                         stderr=errors, start_new_session=True)
            self.terminal_checked = False
            self.terminal_started = time.monotonic()
        except OSError as exc:
            self.lines.put_nowait('日志终端未打开：' + str(exc))

    def _read(self):
        with self.log_path.open('a', buffering=1) as log, self.process.stdout as stream:
            for line in stream:
                log.write(line)
                try:
                    self.lines.put_nowait(line.rstrip())
                except queue.Full:
                    pass
            code = self.process.wait()
            log.write(f'\n任务进程结束，退出码：{code}\n')
        (self.log_dir / 'finished').write_text(str(code))

    def alive(self):
        self.process.poll()  # Reap the leader, but also account for child processes.
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = path.read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == self.process.pid and fields[0] != 'Z':
                    return True
            except (OSError, ValueError, IndexError):
                continue
        return False

    def _signal(self, sig):
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            pass

    def stop(self, automatic=False):
        if not automatic:
            self.stop_requested = True
        if self.deadline is None:
            self._signal(signal.SIGINT)
            self.stage = 1
            self.deadline = time.monotonic() + 45

    def tick(self):
        if (self.log_dir / 'stop-request').exists():
            (self.log_dir / 'stop-request').unlink(missing_ok=True)
            self.stop()
        if self.terminal_process is not None and not self.terminal_checked:
            if (self.log_dir / 'terminal-opened').exists():
                self.terminal_checked = True
            elif time.monotonic() - self.terminal_started > 5:
                self.terminal_checked = True
                try:
                    self.lines.put_nowait('日志终端未确认打开，可点击“打开日志终端”重试；任务仍在运行。')
                except queue.Full:
                    pass
        if self.process.poll() is not None and self.deadline is None and self.alive():
            self.stop(automatic=True)
        if self.deadline is not None and time.monotonic() >= self.deadline:
            self._signal(signal.SIGTERM if self.stage == 1 else signal.SIGKILL)
            self.stage += 1
            self.deadline = time.monotonic() + 8
        return self.alive()


def launch_args(mode, folder, gui=True, rviz=True):
    folder = Path(folder).expanduser().resolve()
    package = 'go2_mid360_sim' if mode == 'simulation' else 'go2_mid360_bringup'
    args = ['ros2', 'launch', package, mode + '.launch.py',
            'gui:=' + str(gui).lower(),
            ('rviz:=' if mode == 'simulation' else 'show_rviz:=') + str(rviz).lower()]
    if mode == 'mapping':
        args += ['map_output:=' + str(folder / 'map.pcd')]
    elif mode == 'localization':
        args += ['map_file:=' + str(folder / 'map.pcd')]
    elif mode == 'navigation':
        args += ['pcd_map:=' + str(folder / 'map.pcd'), 'nav_map:=' + str(folder / 'map.yaml')]
    return args
