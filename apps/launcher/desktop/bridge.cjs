const { spawn } = require('node:child_process');
const readline = require('node:readline');
const { EventEmitter } = require('node:events');

class Bridge extends EventEmitter {
  constructor(python, script, options = {}) {
    super(); this.pending = new Map(); this.seq = 0; this.output = ''; this.session = {state: 'idle'};
    this.child = spawn(python, ['-u', script], { ...options, stdio: ['pipe', 'pipe', 'pipe'] });
    const lines = readline.createInterface({input: this.child.stdout});
    lines.on('line', line => {
      try {
        const message = JSON.parse(line);
        if (message.event) {
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
    this.child.stderr.on('data', data => this.emit('event', {event: 'error', error: data.toString()}));
    const fail = error => {
      this.dead = true;
      for (const r of this.pending.values()) { clearTimeout(r.timer); r.reject(error); }
      this.pending.clear(); this.emit('event', {event: 'error', error: error.message});
    };
    this.child.on('error', fail);
    this.child.on('exit', (code, signal) => fail(new Error(`Backend exited (${code ?? signal})`)));
    this.child.stdin.on('error', error => fail(error));
  }
  request(method, params = {}) {
    if (this.dead) return Promise.reject(new Error('Backend is unavailable'));
    return new Promise((resolve, reject) => {
      const id = ++this.seq;
      const timeout = method === 'stop' || method === 'workbench_stop' || (method === 'workbench_request' && params?.method === 'park') ? 125000 : 15000;
            const timer = setTimeout(() => { this.pending.delete(id); reject(new Error('Backend request timed out')); }, timeout);
      this.pending.set(id, {resolve, reject, timer});
      this.child.stdin.write(JSON.stringify({id, method, params}) + '\n');
    });
  }
  async close() {
    if (this.dead) return;
    this.child.stdin.end();
    await new Promise(resolve => {
      let timer;
      const finish = () => { clearTimeout(timer); resolve(); };
      this.child.once('exit', finish);
      timer = setTimeout(() => { this.child.kill('SIGTERM'); resolve(); }, 7000);
    });
  }
}
module.exports = {Bridge};
