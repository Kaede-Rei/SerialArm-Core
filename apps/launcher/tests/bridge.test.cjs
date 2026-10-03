const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');
const { Bridge } = require('../desktop/bridge.cjs');
const python = process.env.SERIAL_ARM_LAUNCHER_PYTHON || 'python3';
const script = path.resolve(__dirname, '../backend/bridge.py');

test('bridge returns profiles and rejects unknown commands without shell access', async () => {
    const bridge = new Bridge(python, script, { cwd: path.resolve(__dirname, '../../..') });
    try {
        const result = await bridge.request('profiles');
        assert.ok(result.profiles.some(p => p.name === 'dm_arm_gray'));
        await assert.rejects(bridge.request('shell', { command: 'touch /tmp/not-allowed' }), /unknown IPC method/);
        const info = await bridge.request('inspect', { profile: 'dm_arm_gray' });
        assert.equal(info.write_enabled, true);
        await assert.rejects(bridge.request('start', { mode: 'terminal', config: { profile: 'dm_arm_gray' } }), /unavailable|confirmation/);
    } finally { await bridge.close(); }
});

test('pending requests fail promptly when backend exits', async () => {
    const bridge = new Bridge(python, '-c', {});
    // An invalid script path makes the child exit; there is no hanging request
    await assert.rejects(bridge.request('status'), /Backend exited|unavailable/);
    assert.equal(bridge.pending.size, 0);
});
