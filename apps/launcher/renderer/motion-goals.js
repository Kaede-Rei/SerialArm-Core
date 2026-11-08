/* Goal conversion edits parameter drafts only, never robot commands */
(function (root) {
    function returnFactor(zeta) {
        if (!Number.isFinite(zeta) || zeta < .7 || zeta > 1.2) throw new Error('Return damping ratio must be between 0.7 and 1.2');
        const error = time => {
            if (Math.abs(zeta - 1) < 1e-7) return Math.exp(-time) * (1 + time);
            if (zeta < 1) { const w = Math.sqrt(1 - zeta * zeta); return Math.exp(-zeta * time) * (Math.cos(w * time) + zeta / w * Math.sin(w * time)); }
            const d = Math.sqrt(zeta * zeta - 1), a = -zeta + d, b = -zeta - d;
            return (-b * Math.exp(a * time) + a * Math.exp(b * time)) / (a - b);
        };
        // In this damping range the first overshoot stays within the 5% band
        let lower = 0, upper = 30;
        for (let i = 0; i < 60; i++) { const middle = (lower + upper) / 2; if (error(middle) > .05) lower = middle; else upper = middle; }
        return upper;
    }
    function convert(goal) {
        const positive = (value, name) => { if (typeof value !== 'number' || !Number.isFinite(value) || value <= 0) throw new Error(name + ' must be finite and positive'); return value; };
        const time = positive(goal.time, 'Response time');
        if (goal.kind === 'free') {
            const mass = 1 / positive(goal.acceleration, 'Acceleration per unit effort');
            const damping = -Math.log(.05) * mass / time;
            if (![mass, damping].every(value => Number.isFinite(value) && value > 0)) throw new Error('Interaction goal exceeds the finite parameter range');
            return { mass, damping, stiffness: 0 };
        }
        if (goal.kind !== 'return') throw new Error('Unknown interaction goal');
        const stiffness = 1 / positive(goal.compliance, 'Static compliance');
        const omega = returnFactor(goal.zeta) / time;
        const mass = stiffness / (omega * omega), damping = 2 * goal.zeta * Math.sqrt(mass * stiffness);
        if (![mass, damping, stiffness].every(value => Number.isFinite(value) && value >= 0) || mass === 0) throw new Error('Interaction goal exceeds the finite parameter range');
        return { mass, damping, stiffness };
    }
    function fromMdk(mass, damping, stiffness) {
        mass = Number.isFinite(mass) && mass > 0 ? mass : 1;
        damping = Number.isFinite(damping) && damping >= 0 ? damping : 0;
        stiffness = Number.isFinite(stiffness) && stiffness >= 0 ? stiffness : 0;
        const zeta = stiffness > 0 ? damping / (2 * Math.sqrt(mass * stiffness)) : 1;
        const normalized = zeta >= .7 && zeta <= 1.2 ? zeta : 1;
        return { kind: stiffness > 0 ? 'return' : 'free', compliance: stiffness > 0 ? 1 / stiffness : .1,
            time: stiffness > 0 ? returnFactor(normalized) / Math.sqrt(stiffness / mass) : damping > 0 ? -Math.log(.05) * mass / damping : 2,
            zeta: normalized, acceleration: 1 / mass };
    }
    const api = { returnFactor, convert, fromMdk };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.SerialArmMotionGoals = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
