"""Persistent read-only Core model probe used by the embedded model preview."""
import json
import math
from pathlib import Path
import select
import subprocess
import threading

from profiles import config_values


class ModelRuntime:
    def __init__(self):
        self.process = None
        self.key = None
        self.lock = threading.Lock()

    def _close_locked(self):
        process = self.process
        self.process = None
        self.key = None
        if process is None:
            return
        try:
            if process.poll() is None and process.stdin:
                process.stdin.write('quit\n')
                process.stdin.flush()
                process.wait(timeout=1)
        except (BrokenPipeError, OSError, subprocess.TimeoutExpired):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1)
        finally:
            for stream in (process.stdin, process.stdout, process.stderr):
                try:
                    stream and stream.close()
                except OSError:
                    pass

    def close(self):
        with self.lock:
            self._close_locked()

    def _ensure_locked(self, inspector, config, force_restart=False):
        c = config_values(config)
        info = inspector.inspect(c)
        core = info['resources'].get('core')
        if not core or not Path(core).is_file():
            raise ValueError('Core model configuration is unavailable')
        probe = inspector.model_probe()
        if not probe:
            raise ValueError('serial_arm_model_probe is not installed')
        key = (str(Path(probe).resolve()), str(Path(core).resolve()))
        if not force_restart and self.process is not None and self.key == key and self.process.poll() is None:
            return c, info
        self._close_locked()
        self.process = subprocess.Popen(
            [probe, '--config', core, '--server'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.key = key
        return c, info

    @staticmethod
    def _positions_line(positions):
        if positions is None:
            return ''
        if not isinstance(positions, list) or any(
            not isinstance(value, (int, float)) or not math.isfinite(value) for value in positions
        ):
            raise ValueError('positions must be a finite numeric array')
        return ','.join(format(float(value), '.17g') for value in positions)

    def sample(self, inspector, config, positions=None, force_restart=False):
        with self.lock:
            c, info = self._ensure_locked(inspector, config, force_restart)
            process = self.process
            line = self._positions_line(positions)
            try:
                process.stdin.write(line + '\n')
                process.stdin.flush()
            except (BrokenPipeError, OSError):
                self._close_locked()
                raise ValueError('model probe server is unavailable')
            ready, _, _ = select.select([process.stdout], [], [], 3.0)
            if not ready:
                self._close_locked()
                raise ValueError('model probe server timed out')
            output = process.stdout.readline()
            if not output:
                detail = process.stderr.read().strip() if process.stderr else ''
                self._close_locked()
                raise ValueError(detail or 'model probe server exited')
            try:
                native = json.loads(output)
            except json.JSONDecodeError:
                raise ValueError('model probe returned invalid JSON')
            if not native.get('ok'):
                raise ValueError(native.get('error') or 'model probe failed')
            return c, info, native

    def model(self, inspector, config, positions=None):
        c, info, native = self.sample(inspector, config, positions, force_restart=True)
        return inspector.model_payload(c, native, info)

    def preview(self, inspector, config, positions):
        _, _, native = self.sample(inspector, config, positions)
        return native
