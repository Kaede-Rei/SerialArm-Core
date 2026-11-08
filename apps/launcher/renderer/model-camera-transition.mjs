import {stepCameraSpring} from './model-camera-zoom.mjs';

// Orbit in spherical coordinates, rather than drawing a line through the model
// when moving between opposite views. All six components retain their velocity
export class ModelCameraTransition {
  constructor(controls, preference = globalThis.matchMedia?.('(prefers-reduced-motion: reduce)')) {
    this.controls = controls;
    this.camera = controls.object;
    this.preference = preference;
    this.offset = this.camera.position.clone();
    const yUp = this.camera.position.clone().set(0, 1, 0);
    this.toYUp = this.camera.quaternion.clone().setFromUnitVectors(this.camera.up.clone().normalize(), yUp);
    this.fromYUp = this.toYUp.clone().invert();
    this.active = false;
    this.disposed = false;
    this.onInput = () => this.cancel();
    this.onPreference = () => { if (this.preference.matches && this.active) this.finish(); };
    controls.addEventListener('start', this.onInput);
    controls.addEventListener('zoom-input', this.onInput);
    preference?.addEventListener('change', this.onPreference);
  }

  spherical(direction) {
    this.offset.copy(direction).applyQuaternion(this.toYUp);
    const radius = Math.max(this.offset.length(), 1e-6);
    return {theta: Math.atan2(this.offset.x, this.offset.z),
      phi: Math.acos(Math.max(-1, Math.min(1, this.offset.y / radius))), radius};
  }

  clearDamping() {
    // Drain OrbitControls' pending input through its public API, then restore
    // the displayed pose synchronously so no stale drag fights the transition
    const position = this.camera.position.clone(), target = this.controls.target.clone();
    const quaternion = this.camera.quaternion.clone(), damping = this.controls.enableDamping;
    this.controls.enableDamping = false; this.controls.update();
    this.controls.enableDamping = damping;
    this.camera.position.copy(position); this.camera.quaternion.copy(quaternion);
    this.controls.target.copy(target); this.camera.updateMatrixWorld();
  }

  move(direction, distance, target = this.controls.target, immediate = false) {
    if (this.disposed) return;
    this.clearDamping();
    const desired = this.spherical(direction);
    if (!this.active) {
      const shown = this.spherical(this.camera.position.clone().sub(this.controls.target));
      this.axes = [shown.theta, shown.phi, Math.log(shown.radius), ...this.controls.target.toArray()]
        .map(value => ({value, velocity: 0, goal: value}));
    }
    const min = Math.max(this.controls.minDistance, 1e-6), max = this.controls.maxDistance;
    const radius = Math.max(min, Math.min(max, distance));
    const theta = this.axes[0].value;
    const turn = Math.atan2(Math.sin(desired.theta - theta), Math.cos(desired.theta - theta));
    const phi = Math.max(this.controls.minPolarAngle, Math.min(this.controls.maxPolarAngle, desired.phi));
    const goals = [theta + turn, phi, Math.log(radius), ...target.toArray()];
    this.axes.forEach((axis, index) => { axis.goal = goals[index]; });
    this.active = true;
    if (immediate || this.preference?.matches) this.finish();
  }

  apply() {
    const [theta, phi, radius, x, y, z] = this.axes.map(axis => axis.value);
    const min = Math.max(this.controls.minDistance, 1e-6), max = this.controls.maxDistance;
    const length = Math.max(min, Math.min(max, Math.exp(radius)));
    const polar = Math.max(this.controls.minPolarAngle, Math.min(this.controls.maxPolarAngle, phi));
    if (polar !== phi) { this.axes[1].value = polar; this.axes[1].velocity = 0; }
    this.controls.target.set(x, y, z);
    this.offset.set(Math.sin(polar) * Math.sin(theta), Math.cos(polar), Math.sin(polar) * Math.cos(theta))
      .multiplyScalar(length).applyQuaternion(this.fromYUp);
    this.camera.position.copy(this.controls.target).add(this.offset);
    this.camera.lookAt(this.controls.target); this.camera.updateMatrixWorld();
  }

  update(seconds) {
    if (!this.active || this.disposed || !Number.isFinite(seconds) || seconds <= 0) return false;
    if (!this.controls.enabled) { this.cancel(); return false; }
    for (const axis of this.axes) {
      [axis.value, axis.velocity] = stepCameraSpring(axis.value, axis.velocity, axis.goal, seconds);
    }
    if (this.axes.every(axis => Math.abs(axis.value - axis.goal) < 1e-5 && Math.abs(axis.velocity) < 1e-4)) {
      this.finish();
    } else this.apply();
    return true;
  }

  finish() {
    for (const axis of this.axes) { axis.value = axis.goal; axis.velocity = 0; }
    this.apply(); this.active = false;
  }

  cancel() { this.active = false; }

  dispose() {
    this.cancel(); this.disposed = true;
    this.controls.removeEventListener('start', this.onInput);
    this.controls.removeEventListener('zoom-input', this.onInput);
    this.preference?.removeEventListener('change', this.onPreference);
  }
}
