const { spawn } = require('node:child_process');
const readline = require('node:readline');
const { EventEmitter } = require('node:events');
const fs = require('node:fs');

// Linux sessions already use start_new_session in the backend. Retain the
// process start time as well as its PID so local cleanup cannot target a reused
// PID after the backend has disappeared
function processIdentity(pid) {
  if (!Number.isSafeInteger(pid) || pid <= 0) return null;
  try {
    const stat = fs.readFileSync(`/proc/${pid}/stat`, 'utf8');
    const fields = stat.slice(stat.lastIndexOf(')') + 2).trim().split(/\s+/);
    if (!/^\d+$/.test(fields[19] || '')) return null;
    return {pid, state: fields[0], parent: Number(fields[1]), group: Number(fields[2]), session: Number(fields[3]), started: fields[19]};
  } catch (error) { if (['ENOENT', 'ESRCH', 'EACCES', 'EPERM'].includes(error.code)) return null; throw error; }
}
const sameProcess = (a, b) => a && b && a.pid === b.pid && a.started === b.started;

class Bridge extends EventEmitter {
  constructor(python, script, options = {}) {
    super(); this.pending = new Map(); this.seq = 0; this.output = ''; this.session = {state: 'idle'}; this.stderr = ''; this.dead = false;
    this.ownedProcesses = new Map();
    let readyResolve, readyReject;
    this.ready = new Promise((resolve, reject) => { readyResolve = resolve; readyReject = reject; });
    this.ready.catch(() => {});
    this._readyResolve = readyResolve; this._readyReject = readyReject;
    this._readyTimer = setTimeout(() => this._fail(new Error('Backend startup timed out')), 8000);
    this.child = spawn(python, ['-u', script], { ...options, stdio: ['pipe', 'pipe', 'pipe'] });
    this.backendIdentity = processIdentity(this.child.pid);
    const lines = readline.createInterface({input: this.child.stdout});
    this.lines = lines;
    lines.on('line', line => {
      try {
        const message = JSON.parse(line);
        if (message.event) {
          if (message.event === 'backend_ready') {
            clearTimeout(this._readyTimer); this._readyTimer = null;
            if (this._readyResolve) { this._readyResolve(); this._readyResolve = null; this._readyReject = null; }
          }
          if (message.event === 'output') this.output = (this.output + message.data).slice(-1024 * 1024);
          if (message.event === 'session') {
            this.session = message;
            const identity = processIdentity(message.pid);
            if (sameProcess(this.backendIdentity, processIdentity(this.child.pid)) && identity?.parent === this.child.pid && identity.group === identity.pid && identity.session === identity.pid)
              this.ownedProcesses.set(identity.pid, identity);
          }
          this.emit('event', message); return;
        }
        const request = this.pending.get(message.id);
        if (!request) return;
        clearTimeout(request.timer); this.pending.delete(message.id);
        if (message.error) {
          const details = typeof message.error === 'object' && message.error !== null
            ? message.error : {code: 'LEGACY_BACKEND_ERROR', message: String(message.error)};
          const failure = new Error(details.message || details.technical || '后台请求未完成');
          failure.serialarm = details;
          request.reject(failure);
        } else request.resolve(message.result);
      } catch (error) { this.emit('event', {event: 'error', error: 'Invalid backend message: ' + error.message}); }
    });
    this.child.stderr.on('data', data => {
      const detail = data.toString();
      this.stderr = (this.stderr + detail).slice(-64 * 1024);
      process.stderr.write(`[SerialArm backend] ${detail}`);
      // stderr is a diagnostic log channel, not an operator failure event
    });
    this.child.on('error', error => this._fail(error));
    this.child.on('exit', (code, signal) => {
      if (this.closing) return;
      const detail = this.stderr.trim();
      this._fail(new Error(`Backend exited (${code ?? signal})${detail ? `: ${detail}` : ''}`));
    });
    this.child.stdin.on('error', error => this._fail(
      error.code === 'EPIPE' ? new Error('Backend exited/unavailable: stdin closed (EPIPE)' + (this.stderr ? ': ' + this.stderr.trim() : '')) : error));
  }
  _fail(error) {
    if (this.dead) return;
    this.captureOwnedProcesses();
    this.dead = true;
    clearTimeout(this._readyTimer); this._readyTimer = null;
    if (this._readyReject) { this._readyReject(error); this._readyResolve = null; this._readyReject = null; }
    for (const r of this.pending.values()) { clearTimeout(r.timer); r.reject(error); }
    this.pending.clear(); this.emit('event', {event: 'error', error: error.message});
  }
  captureOwnedProcesses() {
    const processes = new Map();
    for (const name of fs.readdirSync('/proc')) {
      if (!/^\d+$/.test(name)) continue;
      const identity = processIdentity(Number(name));
      if (identity) processes.set(identity.pid, identity);
    }
    const known = new Map([...this.ownedProcesses].filter(([pid, identity]) => sameProcess(identity, processes.get(pid))));
    if (sameProcess(this.backendIdentity, processes.get(this.child.pid))) known.set(this.child.pid, this.backendIdentity);
    const sessions = new Set([...known.values()].filter(p => p.group === p.pid && p.session === p.pid).map(p => p.pid));
    let added;
    do {
      added = false;
      for (const identity of processes.values()) {
        if (!known.has(identity.pid) && (known.has(identity.parent) || sessions.has(identity.session))) {
          known.set(identity.pid, identity); added = true;
        }
      }
    } while (added);
    for (const [pid, identity] of known) this.ownedProcesses.set(pid, identity);
  }
  async forceClose() {
    if (!this.dead) throw new Error('Local cleanup is only available after the backend has disconnected');
    this.captureOwnedProcesses(); this.closing = true;
    const targets = [...this.ownedProcesses.values()].filter(identity => sameProcess(identity, processIdentity(identity.pid)));
    // Terminate only observed members of this launcher's process tree/session,
    // never a negative PID or an unverified process group
    for (const identity of targets.sort((a, b) => (a.pid === this.child.pid) - (b.pid === this.child.pid))) {
      if (!sameProcess(identity, processIdentity(identity.pid))) continue;
      try { process.kill(identity.pid, 'SIGKILL'); }
      catch (error) { if (error.code !== 'ESRCH') throw error; }
    }
    const deadline = Date.now() + 1500;
    while (targets.some(identity => { const live = processIdentity(identity.pid); return sameProcess(identity, live) && !['Z', 'X'].includes(live.state); })) {
      if (Date.now() >= deadline) throw new Error('Known launcher processes have not exited after local cleanup');
      await new Promise(resolve => setTimeout(resolve, 25));
    }
    this.lines.close();
    for (const stream of [this.child.stdin, this.child.stdout, this.child.stderr]) stream.destroy();
    return {terminatedPids: targets.map(identity => identity.pid), force_stop_unverified: true};
  }
  request(method, params = {}) {
    if (this.dead) {
      const detail = this.stderr.trim();
      return Promise.reject(new Error(detail ? `Backend is unavailable: ${detail}` : 'Backend is unavailable'));
    }
    return new Promise((resolve, reject) => {
      const id = ++this.seq;
      const timeout = ['model_calibration_idim','model_calibration_idim_audit','model_calibration_idim_export','model_calibration_advanced'].includes(method) ? 340000 : (method === 'model_calibration_export_full_inertial' || method === 'model_calibration_advanced') ? 190000 : method === 'model_calibration_recompute' || method === 'model_calibration_export_urdf' || method === 'calibration_tool_export' ? 190000 : method === 'stop' || method === 'workbench_stop' || method === 'close' || (method === 'workbench_request' && params?.method === 'park') ? 125000 : (method === 'workbench_request' && params?.method === 'model_calibration_teach_stop' ? 60000 : (method === 'workbench_request' && params?.method === 'model_calibration_import_trajectory' ? 60000 : 30000));
      const timer = setTimeout(() => { this.pending.delete(id); reject(new Error('Backend request timed out')); }, timeout);
      this.pending.set(id, {resolve, reject, timer});
      this.child.stdin.write(JSON.stringify({id, method, params}) + '\n');
    });
  }
  async close() {
    if (this.dead) return;
    if (['running', 'stopping'].includes(this.session.state)) throw new Error('Stop the active session before closing the backend');
    this.closing = true;
    try {
      await this.request('close');
      await new Promise((resolve, reject) => {
        if (this.child.exitCode !== null || this.child.signalCode !== null) return resolve();
        const finish = () => { clearTimeout(timer); resolve(); };
        const timer = setTimeout(() => { this.child.off('exit', finish); reject(new Error('Backend has not exited; close was not forced')); }, 7000);
        this.child.once('exit', finish);
      });
      this.dead = true;
    } catch (error) {
      this.closing = false;
      throw error;
    }
  }
}
module.exports = {Bridge};
