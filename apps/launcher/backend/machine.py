"""Structured native workspace session with bounded JSON line transport and device leases"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid
from collections import deque


class NativeSession:
    def __init__(self, emit, lock_dir):
        self.emit = emit
        self.lock_dir = Path(lock_dir)
        self.proc = None
        self.pending = {}
        self.leases = []
        self.seq = 0
        self.state = {'state': 'idle', 'mode': 'workbench'}
        self.mutex = threading.RLock()
        self.reader = None
        self.stderr_reader = None
        self.telemetry_history = deque(maxlen=20000)
        self.stderr_tail = deque(maxlen=32)

    def status(self):
        with self.mutex:
            return dict(self.state)

    def _lease(self, devices):
        self.lock_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        for device in sorted(set(os.path.realpath(d) for d in devices)):
            name = hashlib.sha256(device.encode()).hexdigest() + '.lock'
            lease = (self.lock_dir / name).open('a+')
            try:
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lease.close()
                self._release()
                raise ValueError('device is owned by another Launcher: ' + device)
            self.leases.append(lease)

    def _release(self):
        for lease in self.leases:
            try: fcntl.flock(lease, fcntl.LOCK_UN)
            finally: lease.close()
        self.leases = []

    def start(self, argv, env, devices):
        with self.mutex:
            if self.state.get('state') in ('running', 'stopping'):
                raise ValueError('a workspace session is already running')
            if self.proc:
                for stream in (self.proc.stdin, self.proc.stdout, self.proc.stderr):
                    stream.close()
            self._lease(devices)
            try:
                self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                             env=env, text=True, bufsize=1, start_new_session=True)
            except BaseException:
                self._release()
                raise
            self.telemetry_history.clear()
            self.stderr_tail.clear()
            self.state = {'state': 'running', 'mode': 'workbench', 'recovery_required': False, 'pid': self.proc.pid,
                          'session_id':uuid.uuid4().hex, 'started_at': time.time(), 'devices': list(devices), 'command': list(argv)}
            self.reader = threading.Thread(target=self._read_stdout, args=(self.proc,self.state['session_id']), daemon=True)
            self.stderr_reader = threading.Thread(target=self._read_stderr, args=(self.proc,self.state['session_id']), daemon=True)
            self.reader.start(); self.stderr_reader.start()
        # A spawned terminal can still fail while loading its config or native
        # libraries. Only report a connected workspace after the IPC is ready.
        try:
            self.request('status')
        except BaseException:
            # Startup has only sent a read-only status request, so the terminal
            # has not received any activation or motion command. Close its
            # input, then stop a stalled initializer before allowing a retry.
            proc = self.proc
            try: proc.stdin.close()
            except BrokenPipeError: pass
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                try: os.killpg(proc.pid, 9)
                except ProcessLookupError: pass
                proc.wait(timeout=2)
            self.reader.join(timeout=2)
            self.stderr_reader.join(timeout=1)
            raise
        with self.mutex:
            if self.state.get('state') != 'running':
                raise ValueError(self.state.get('exit_error') or 'native process exited during startup')
            self.emit({'event': 'session', **self.state})
            return self.status()

    def _read_stdout(self, proc, session_id):
        try:
            for line in proc.stdout:
                try: message = json.loads(line)
                except json.JSONDecodeError:
                    self.emit({'event': 'error', 'error': 'Invalid native session message'})
                    continue
                if message.get('event'):
                    if message.get('event') == 'telemetry':
                        with self.mutex:
                            self.telemetry_history.append(message)
                    self.emit({**message,'session_id':session_id,'session_pid':proc.pid})
                    continue
                request = None
                with self.mutex: request = self.pending.pop(message.get('id'), None)
                if request:
                    request['message'] = message
                    request['event'].set()
        finally:
            code = proc.wait()
            # Drain diagnostics before resolving pending requests so startup
            # failures carry the cause instead of only "session closed".
            if self.stderr_reader:
                self.stderr_reader.join(timeout=1)
            with self.mutex:
                diagnostic = '\n'.join(self.stderr_tail).strip()[-8192:]
                error = f'native process exited (code {code})'
                if diagnostic:
                    error += ': ' + diagnostic
                for request in self.pending.values():
                    request['message'] = {'error': error}
                    request['event'].set()
                self.pending.clear()
                self._release()
                self.state.update(state='exited', exit_code=code)
                if code:
                    self.state['exit_error'] = error
                self.emit({'event': 'session', **self.state})

    def _read_stderr(self, proc, session_id):
        for line in proc.stderr:
            if line.strip():
                with self.mutex:
                    self.stderr_tail.append(line[-8192:].rstrip())
                self.emit({'event': 'native_log', 'data': line[-8192:],
                           'session_id': session_id, 'session_pid': proc.pid})

    def request(self, method, params=None, timeout=8, cancel_event=None):
        if not isinstance(method, str) or not method or len(method) > 128: raise ValueError('invalid native request method')
        if params is None: params = {}
        if not isinstance(params, dict): raise ValueError('native request params must be an object')
        with self.mutex:
            if not self.proc or self.state.get('state') not in ('running', 'stopping'):
                raise ValueError(self.state.get('exit_error') or 'workspace session is not running')
            if cancel_event is not None and cancel_event.is_set():
                raise ValueError('park interrupted by operator')
            # A timed-out command may still execute later in the native process
            # Never queue another hardware action until the operator reconnects
            if self.state.get('recovery_required'):
                raise RuntimeError('native session is unresponsive; force disconnect after making hardware safe')
            self.seq += 1
            number = self.seq
            event = threading.Event()
            request = {'event': event, 'message': None}
            self.pending[number] = request
            try:
                self.proc.stdin.write(json.dumps({'id': number, 'method': method, 'params': params}, separators=(',', ':')) + '\n')
                self.proc.stdin.flush()
            except BaseException:
                self.pending.pop(number, None)
                raise
        if not event.wait(timeout):
            with self.mutex:
                # The transport timeout does not cancel a native hardware call
                # Treat its result and the current actuator state as unknown
                self.pending.pop(number, None)
                if self.state.get('state') == 'running':
                    self.state['recovery_required'] = True
                    self.state['recovery_reason'] = method
                    self.emit({'event': 'session', **self.state})
            raise TimeoutError('native session request timed out; state unknown, force disconnect after making hardware safe')
        message = request['message'] or {}
        if message.get('error'): raise ValueError(message['error'])
        return message.get('result')


    def telemetry(self):
        with self.mutex:
            return list(self.telemetry_history)

    def note_disconnect_failure(self, error):
        with self.mutex:
            if self.state.get('state') not in ('running','stopping'):
                return
            self.state.update(disconnect_blocked=True, disconnect_error=str(error))
            self.emit({'event':'session', **self.state})

    def stop(self, force=False, cancel_event=None):
        with self.mutex:
            if not self.proc or self.state.get('state') not in ('running', 'stopping'): return self.status()
            proc = self.proc
        if not force:
            if self.state.get('state') == 'running':
                # A native rejection must preserve the connection and its leases
                # Only an explicit force request may terminate the process
                try:
                    self.request('shutdown', timeout=120, cancel_event=cancel_event)
                except Exception as error:
                    self.note_disconnect_failure(error)
                    raise
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired as error:
                with self.mutex:
                    self.state['recovery_required'] = True
                    self.state['recovery_reason'] = 'shutdown'
                    self.emit({'event': 'session', **self.state})
                raise TimeoutError('native shutdown did not exit; connection preserved, check hardware before forcing disconnect') from error
            return self.status()
        with self.mutex:
            if self.state.get('state') != 'exited':
                self.state['state'] = 'stopping'
                self.state['force_stop_unverified'] = True
                self.emit({'event': 'session', **self.state})
        if proc.poll() is None:
            try: os.killpg(proc.pid, 9)
            except ProcessLookupError: pass
        return self.status()

    def close(self):
        self.stop()
        if self.reader: self.reader.join(timeout=6)
        if self.stderr_reader: self.stderr_reader.join(timeout=2)
        if self.proc and self.proc.poll() is None:
            raise TimeoutError('native session is still running; connection preserved')
        if self.proc:
            for stream in (self.proc.stdin, self.proc.stdout, self.proc.stderr):
                try: stream.close()
                except Exception: pass
