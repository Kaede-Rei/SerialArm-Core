/* Passive response measurements use the native cycle clock and explicit release marks. */
(function (root) {
    const finite = value => typeof value === 'number' && Number.isFinite(value);
    const mean = values => values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : null;
    const at = (values, index) => finite(values?.[index]) ? values[index] : null;
    const limited = value => Array.isArray(value) ? value.some(flag => typeof flag === 'number' && flag !== 0) : value === true;
    function sample(snapshot) {
        if (snapshot?.valid !== true || !finite(snapshot.monotonic_ns) || !Array.isArray(snapshot.joint_names)) return null;
        const joint = snapshot.joint || {};
        return {
            time_s: snapshot.monotonic_ns / 1e9, sequence: snapshot.sequence,
            feedback_receive_ns: finite(snapshot.feedback_receive_ns) ? snapshot.feedback_receive_ns : null,
            pos: snapshot.joint_names.map((_, i) => at(joint.pos, i)),
            vel: snapshot.joint_names.map((_, i) => at(joint.vel, i)),
            acc: snapshot.joint_names.map((_, i) => at(joint.acc, i)),
            nominal: snapshot.joint_names.map((_, i) => at(joint.nominal_ref_pos, i)),
            nominal_vel: snapshot.joint_names.map((_, i) => at(joint.nominal_ref_vel, i)),
            nominal_acc: snapshot.joint_names.map((_, i) => at(joint.nominal_ref_acc, i)),
            virtual: snapshot.joint_names.map((_, i) => at(joint.virtual_ref_pos, i)),
            virtual_vel: snapshot.joint_names.map((_, i) => at(joint.virtual_ref_vel, i)),
            virtual_acc: snapshot.joint_names.map((_, i) => at(joint.virtual_ref_acc, i)),
            accepted: snapshot.joint_names.map((_, i) => at(joint.accepted_ref_pos, i)),
            accepted_vel: snapshot.joint_names.map((_, i) => at(joint.accepted_ref_vel, i)),
            accepted_acc: snapshot.joint_names.map((_, i) => at(joint.accepted_ref_acc, i)),
            measured_torque: snapshot.joint_names.map((_, i) => at(joint.tor, i)),
            pd_torque: snapshot.joint_names.map((_, i) => at(joint.pd_torque, i)),
            model_feedforward: snapshot.joint_names.map((_, i) => at(joint.model_feedforward, i)),
            predicted_total_torque: snapshot.joint_names.map((_, i) => at(joint.predicted_total_torque, i)),
            effort_limit: snapshot.joint_names.map((_, i) => at(joint.effort_limit, i)),
            kp: snapshot.joint_names.map((_, i) => at(snapshot.effective_kp || joint.kp, i)),
            kd: snapshot.joint_names.map((_, i) => at(snapshot.effective_kd || joint.kd, i)),
            hold_target: snapshot.joint_names.map((_, i) => at(snapshot.hold_target_pos, i)),
            hold_integral_torque: snapshot.joint_names.map((_, i) => at(snapshot.hold_integral_torque, i)),
            external: snapshot.joint_names.map((_, i) => snapshot.observer_valid === true ? at(joint.tau_ext_hat, i) : null),
            observer_valid: snapshot.observer_valid === true,
            new_measurement: snapshot.new_measurement === true,
            feedback_age_ms: finite(snapshot.feedback_age_ms) ? snapshot.feedback_age_ms : null,
            measurement_dt: finite(snapshot.measurement_dt) ? snapshot.measurement_dt : null,
            feedback_skew_ms: finite(snapshot.feedback_skew_ms) ? snapshot.feedback_skew_ms : null,
            reference_limited: limited(snapshot.reference_limited),
            torque_limited: limited(snapshot.torque_limited),
            reference_limit_flags: Array.isArray(snapshot.reference_limited) ? snapshot.reference_limited.slice() : [],
            torque_limit_flags: Array.isArray(snapshot.torque_limited) ? snapshot.torque_limited.slice() : [],
        };
    }
    function summarize(samples, releaseTime, tolerance = 0.01) {
        const last = samples.at(-1), first = samples[0];
        if (!first) return { samples: 0, duration_s: 0, joints: [] };
        const tolerances = Array.isArray(tolerance) ? tolerance : first.pos.map(() => tolerance);
        if (tolerances.length !== first.pos.length || !tolerances.every(value => finite(value) && value > 0))
            throw new Error('Each joint response tolerance must be finite and positive');
        const joints = first.pos.map((_, i) => {
            const band = tolerances[i];
            const tracking = samples.filter(s => finite(s.pos[i]) && finite(s.accepted[i])).map(s => s.pos[i] - s.accepted[i]);
            const offset = samples.filter(s => finite(s.pos[i]) && finite(s.nominal[i])).map(s => ({ time_s: s.time_s, value: s.pos[i] - s.nominal[i], nominal: s.nominal[i] }));
            const post = finite(releaseTime) ? offset.filter(s => s.time_s >= releaseTime) : [];
            const final = post.filter(s => s.time_s >= last.time_s - 0.5);
            const nominalMoving = post.length > 1 && Math.max(...post.map(s => s.nominal)) - Math.min(...post.map(s => s.nominal)) > band;
            let settling = null;
            if (post.length > 1 && !nominalMoving) {
                let start = post.length - 1;
                while (start >= 0 && Math.abs(post[start].value) <= band) --start;
                const entering = post[start + 1];
                if (entering && post.at(-1).time_s - entering.time_s >= 0.5) settling = entering.time_s - releaseTime;
            }
            const releaseOffset = post[0]?.value;
            const external = samples.map(s => at(s.external, i)).filter(finite);
            return {
                tracking_rms: tracking.length ? Math.sqrt(mean(tracking.map(x => x * x))) : null,
                tracking_max: tracking.length ? Math.max(...tracking.map(Math.abs)) : null,
                peak_deflection: offset.length ? Math.max(...offset.map(s => Math.abs(s.value))) : null,
                peak_external_nm: external.length ? Math.max(...external.map(Math.abs)) : null,
                final_offset: mean(final.map(s => s.value)),
                rebound: finite(releaseOffset) && Math.abs(releaseOffset) > band ? Math.max(0, ...post.map(s => -Math.sign(releaseOffset) * s.value)) : null,
                post_release_change: post.length > 1 ? post.at(-1).value - post[0].value : null,
                settling_s: settling, nominal_moving: nominalMoving,
            };
        });
        return {
            samples: samples.length, duration_s: last.time_s - first.time_s,
            observer_valid_ratio: samples.filter(s => s.observer_valid).length / samples.length,
            reference_limited_ratio: samples.filter(s => s.reference_limited).length / samples.length,
            torque_limited_ratio: samples.filter(s => s.torque_limited).length / samples.length,
            release_time_s: finite(releaseTime) ? releaseTime - first.time_s : null,
            tolerance, stable_dwell_s: 0.5, joints,
        };
    }
    const api = { sample, summarize };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.SerialArmResponse = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
