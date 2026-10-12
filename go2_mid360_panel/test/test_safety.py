from types import SimpleNamespace
from pathlib import Path
import os
import signal
import subprocess
import sys
import time
from go2_mid360_panel.teleop import conflict_reason
from go2_mid360_panel.rviz_recovery import owned_rviz, same_process, process_info
from go2_mid360_panel.processes import Job

class Graph:
    def __init__(self, names=(), publishers=()):
        self.names, self.publishers = names, publishers
    def get_node_names_and_namespaces(self):
        return [(name, '/') for name in self.names]
    def get_publishers_info_by_topic(self, topic):
        assert topic == '/cmd_vel'
        return [SimpleNamespace(node_name=name, node_namespace='/') for name in self.publishers]

def test_navigation_and_other_sources_exclude_keyboard():
    assert conflict_reason(Graph(names=['controller_server']))
    assert '/external_teleop' in conflict_reason(Graph(publishers=['external_teleop']))
    assert not conflict_reason(Graph(publishers=['own']), 'own')
    assert conflict_reason(Graph(publishers=['own', 'other']), 'own')

def test_rviz_recovery_owns_only_its_launch(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    executable = tmp_path/'rviz2'
    executable.symlink_to(sys.executable)
    marker = tmp_path/'runs'
    child_code = f"from pathlib import Path; import time; p=Path({str(marker)!r}); p.open('a').write('run\\n'); time.sleep(60)"
    argv = [str(executable), '-c', child_code]
    code = f'import subprocess,time; subprocess.Popen({argv!r}); time.sleep(60)'
    owner = Job([sys.executable, '-c', code], tmp_path)
    unrelated = subprocess.Popen(argv, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    recovery = None
    try:
        end=time.monotonic()+5
        while not owned_rviz(owner.process.pid) and time.monotonic()<end:time.sleep(.05)
        old = owned_rviz(owner.process.pid)
        assert len(old)==1
        recovery=Job([sys.executable, '-m', 'go2_mid360_panel.rviz_recovery',str(owner.process.pid),*argv], tmp_path)
        while same_process(old[0][0],old[0][1]) and time.monotonic()<end:time.sleep(.05)
        assert not same_process(old[0][0],old[0][1])
        time.sleep(.3)
        assert owner.alive() and unrelated.poll() is None
        assert recovery.alive()
        owner.stop()
        while owner.tick() and time.monotonic()<end:time.sleep(.05)
        while recovery.tick() and time.monotonic()<end:time.sleep(.05)
        assert not recovery.alive()
        assert unrelated.poll() is None
    finally:
        for job in (owner,recovery):
            if job:job._signal(signal.SIGKILL);job.process.wait(timeout=5)
        unrelated.kill();unrelated.wait()


def test_publisher_discovery_waits_then_identifies_own_endpoint(monkeypatch):
    from go2_mid360_panel.teleop import PublisherGuard
    now = [10.]
    monkeypatch.setattr('go2_mid360_panel.teleop.time.monotonic', lambda: now[0])
    graph = Graph()
    endpoints = []
    graph.get_publishers_info_by_topic = lambda topic: endpoints
    def endpoint(name, gid):
        return SimpleNamespace(node_name=name, node_namespace='/', endpoint_gid=[gid]*24)
    guard = PublisherGuard('own')
    assert guard.check(graph) == ('', False)
    endpoints[:] = [endpoint('_NODE_NAME_UNKNOWN_', 1)]
    assert guard.check(graph) == ('', False)
    now[0] += .4
    endpoints[:] = [endpoint('own', 1)]
    assert guard.check(graph) == ('', True)
    endpoints[:] = [endpoint('_NODE_NAME_UNKNOWN_', 1)]
    assert guard.check(graph) == ('', True)  # Same endpoint, temporary loss of name.
    endpoints.append(endpoint('_NODE_NAME_UNKNOWN_', 2))
    assert guard.check(graph) == ('', False)  # Unknown foreign endpoint blocks input.
    now[0] += 2.1
    reason, ready = guard.check(graph)
    assert 'timed out' in reason and not ready
    endpoints[:] = [endpoint('own', 1), endpoint('external', 2)]
    reason, ready = guard.check(graph)
    assert '/external' in reason and not ready
    endpoints[:] = [endpoint('own', 1)]
    assert guard.check(graph) == ('', True)
    graph.names = ['controller_server']
    assert guard.check(graph)[0].startswith('Navigation is running')
