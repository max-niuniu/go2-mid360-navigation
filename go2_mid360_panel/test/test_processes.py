import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from go2_mid360_panel.processes import Job, launch_args


def test_paths_are_single_arguments(tmp_path):
    folder = tmp_path / 'map with spaces; literal'
    for mode, key in [('mapping', 'map_output'), ('localization', 'map_file'), ('navigation', 'pcd_map')]:
        args = launch_args(mode, folder, False, False)
        assert key + ':=' + str(folder/'map.pcd') in args
        assert 'gui:=false' in args
    assert 'rviz:=false' in launch_args('simulation', folder, False, False)


def test_stop_cleans_child_group_and_preserves_unrelated_process(tmp_path):
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], start_new_session=True)
    child_file = tmp_path/'child.pid'
    code = ('import subprocess,sys,time,pathlib; '
            "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
            f'pathlib.Path({str(child_file)!r}).write_text(str(p.pid)); '
            'time.sleep(60)')
    job = Job([sys.executable, '-c', code], tmp_path)
    try:
        limit = time.monotonic()+5
        while not child_file.exists() and time.monotonic()<limit: time.sleep(.05)
        assert child_file.exists()
        job.stop()
        while job.tick() and time.monotonic()<limit: time.sleep(.05)
        assert not job.alive()
        assert unrelated.poll() is None
    finally:
        job._signal(signal.SIGKILL)
        job.process.wait(timeout=5)
        unrelated.terminate(); unrelated.wait(timeout=5)


def test_terminal_interrupt_and_close_behavior(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    job = Job([sys.executable, '-c', "import time; print('test-output',flush=True); time.sleep(60)"], tmp_path)
    viewer = None
    try:
        command = [sys.executable, '-m', 'go2_mid360_panel.terminal', str(job.log_dir)]
        viewer = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic()+5
        while not (job.log_dir/'terminal-opened').exists() and time.monotonic()<deadline: time.sleep(.02)
        assert (job.log_dir/'terminal-opened').exists()
        # Closing the viewer does not stop the task.
        viewer.terminate(); viewer.wait(timeout=5)
        assert job.alive() and not (job.log_dir/'stop-request').exists()
        (job.log_dir/'terminal-opened').unlink()
        viewer = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic()+5
        while not (job.log_dir/'terminal-opened').exists() and time.monotonic()<deadline: time.sleep(.02)
        assert (job.log_dir/'terminal-opened').exists()
        viewer.send_signal(signal.SIGINT)
        while job.tick() and time.monotonic()<deadline: time.sleep(.02)
        assert not job.alive() and job.stop_requested
        output, _ = viewer.communicate('\n', timeout=5)
        assert 'test-output' in output and '已请求停止' in output and '任务已结束' in output
        assert 'test-output' in job.log_path.read_text()
    finally:
        job._signal(signal.SIGKILL); job.process.wait(timeout=5)
        if viewer is not None and viewer.poll() is None:
            viewer.terminate(); viewer.wait(timeout=5)


def test_crashed_parent_cleanup_is_not_a_user_stop(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    job = Job([sys.executable, '-c', "import subprocess,sys; subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); sys.exit(7)"], tmp_path)
    try:
        deadline=time.monotonic()+5
        while job.process.poll() is None and time.monotonic()<deadline: time.sleep(.02)
        while job.tick() and time.monotonic()<deadline: time.sleep(.02)
        assert not job.alive() and job.process.poll()==7
        assert not job.stop_requested
    finally:
        job._signal(signal.SIGKILL);job.process.wait(timeout=5)
