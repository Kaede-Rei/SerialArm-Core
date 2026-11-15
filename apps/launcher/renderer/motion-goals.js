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
        const validMass=typeof mass==='number'&&Number.isFinite(mass)&&mass>0;
        const validDamping=typeof damping==='number'&&Number.isFinite(damping)&&damping>=0;
        const validStiffness=typeof stiffness==='number'&&Number.isFinite(stiffness)&&stiffness>=0;
        const returning=validStiffness&&stiffness>0;
        const zeta=validMass&&validDamping&&returning?damping/(2*Math.sqrt(mass*stiffness)):1;
        const supported=zeta>=.7&&zeta<=1.2;
        return {kind:returning?'return':'free',compliance:returning?1/stiffness:NaN,
            time:validMass&&validDamping?(returning?(supported?returnFactor(zeta)/Math.sqrt(stiffness/mass):NaN):(damping>0?-Math.log(.05)*mass/damping:NaN)):NaN,
            zeta,acceleration:validMass?1/mass:NaN};
    }
    const api = { returnFactor, convert, fromMdk };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.SerialArmMotionGoals = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
