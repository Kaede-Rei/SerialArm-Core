const { spawn } = require('node:child_process');
const readline = require('node:readline');
const { EventEmitter } = require('node:events');

class Bridge extends EventEmitter {
  constructor(python, script, options = {}) {
    super(); this.pending = new Map(); this.seq = 0; this.output = ''; this.session = {state: 'idle'}; this.stderr = ''; this.dead = false;
    let readyResolve, readyReject;
    this.ready = new Promise((resolve, reject) => { readyResolve = resolve; readyReject = reject; });
    this.ready.catch(() => {});
    this._readyResolve = readyResolve; this._readyReject = readyReject;
    this._readyTimer = setTimeout(() => this._fail(new Error('Backend startup timed out')), 8000);
    this.child = spawn(python, ['-u', script], { ...options, stdio: ['pipe', 'pipe', 'pipe'] });
    const lines = readline.createInterface({input: this.child.stdout});
    lines.on('line', line => {
      try {
        const message = JSON.parse(line);
        if (message.event) {
          if (message.event === 'backend_ready') {
            clearTimeout(this._readyTimer); this._readyTimer = null;
            if (this._readyResolve) { this._readyResolve(); this._readyResolve = null; this._readyReject = null; }
          }
          if (message.event === 'output') this.output = (this.output + message.data).slice(-1024 * 1024);
          if (message.event === 'session') this.session = message;
          this.emit('event', message); return;
        }
        const request = this.pending.get(message.id);
        if (!request) return;
        clearTimeout(request.timer); this.pending.delete(message.id);
        message.error ? request.reject(new Error(message.error)) : request.resolve(message.result);
      } catch (error) { this.emit('event', {event: 'error', error: 'Invalid backend message: ' + error.message}); }
    });
    this.child.stderr.on('data', data => {
      const detail = data.toString();
      this.stderr = (this.stderr + detail).slice(-64 * 1024);
      process.stderr.write(`[SerialArm backend] ${detail}`);
      this.emit('event', {event: 'error', error: detail});
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
    this.dead = true;
    clearTimeout(this._readyTimer); this._readyTimer = null;
    if (this._readyReject) { this._readyReject(error); this._readyResolve = null; this._readyReject = null; }
    for (const r of this.pending.values()) { clearTimeout(r.timer); r.reject(error); }
    this.pending.clear(); this.emit('event', {event: 'error', error: error.message});
  }
  request(method, params = {}) {
    if (this.dead) {
      const detail = this.stderr.trim();
      return Promise.reject(new Error(detail ? `Backend is unavailable: ${detail}` : 'Backend is unavailable'));
    }
    return new Promise((resolve, reject) => {
      const id = ++this.seq;
      const timeout = (method === 'model_calibration_export_full_inertial' || method === 'model_calibration_advanced') ? 150000 : method === 'model_calibration_recompute' || method === 'model_calibration_export_urdf' ? 90000 : method === 'stop' || method === 'workbench_stop' || (method === 'workbench_request' && params?.method === 'park') ? 125000 : (method === 'workbench_request' && params?.method === 'model_calibration_teach_stop' ? 35000 : 15000);
      const timer = setTimeout(() => { this.pending.delete(id); reject(new Error('Backend request timed out')); }, timeout);
      this.pending.set(id, {resolve, reject, timer});
      this.child.stdin.write(JSON.stringify({id, method, params}) + '\n');
    });
  }
  async close() {
    if (this.dead) return;
    this.closing = true;
    this.child.stdin.end();
    await new Promise(resolve => {
      let timer;
      const finish = () => { clearTimeout(timer); resolve(); };
      this.child.once('exit', finish);
      timer = setTimeout(() => { this.child.kill('SIGTERM'); resolve(); }, 7000);
    });
    this.dead = true;
  }
}
module.exports = {Bridge};
