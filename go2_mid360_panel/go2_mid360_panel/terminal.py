"""Follow one task's log in a desktop terminal; Ctrl+C requests a managed stop."""
from pathlib import Path
import os
import signal
import sys
import time


def follow(directory):
    directory = Path(directory)
    print('Go2 运行日志 | Ctrl+C 停止任务；关闭此窗口只关闭日志。', flush=True)
    (directory / 'terminal-opened').write_text(str(os.getpid()))

    def interrupt(*_):
        if (directory / 'finished').exists():
            raise SystemExit(0)
        (directory / 'stop-request').touch()
        print('\n已请求停止，等待节点退出与地图保存…', flush=True)

    signal.signal(signal.SIGINT, interrupt)
    with (directory / 'output.log').open(errors='replace') as stream:
        while True:
            output = stream.read()
            if output:
                print(output, end='', flush=True)
            if (directory / 'finished').exists():
                # The writer flushes and closes the log before marking completion.
                print(stream.read(), end='', flush=True)
                break
            time.sleep(.1)
    try:
        input('\n任务已结束。按 Enter 关闭此终端。')
    except (EOFError, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    follow(sys.argv[1])
