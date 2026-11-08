// Exact solution of the shared 480 / 44 / 1 camera spring (roots -20, -24)
export function stepCameraSpring(value, velocity, goal, seconds) {
  const offset = value - goal;
  const slow = (velocity + 24 * offset) / 4, fast = offset - slow;
  const a = slow * Math.exp(-20 * seconds), b = fast * Math.exp(-24 * seconds);
  return [goal + a + b, -20 * a - 24 * b];
}

// Match AT-System-Integration's canvas response: stiffness 480, damping 44,
// mass 1, and 0.0025 log-scale units per normalized wheel pixel
export class SmoothModelZoom {
  constructor(controls, preference = globalThis.matchMedia?.('(prefers-reduced-motion: reduce)')) {
    this.controls = controls;
    this.element = controls.domElement;
    this.camera = controls.object;
    this.preference = preference;
    this.pointer = {x: 0, y: 0};
    this.ray = this.camera.position.clone();
    this.direction = this.camera.position.clone();
    this.dragging = false;
    this.disposed = false;
    this.cancel();
    this.onWheel = event => this.wheel(event);
    this.onStart = () => { this.dragging = true; this.cancel(); };
    this.onEnd = () => { this.dragging = false; };
    this.onPreference = () => {
      if (this.preference.matches && this.active) {
        this.applyDistance(Math.exp(this.goal));
        this.cancel();
        this.controls.update();
      }
    };
    // Capture only wheel zoom; OrbitControls retains rotation, pan and touch
    this.element.addEventListener('wheel', this.onWheel, {capture: true, passive: false});
    controls.addEventListener('start', this.onStart);
    controls.addEventListener('end', this.onEnd);
    preference?.addEventListener('change', this.onPreference);
  }

  limits() {
    return [Math.max(this.controls.minDistance, 1e-6), this.controls.maxDistance];
  }

  cancel() {
    this.value = this.goal = Math.log(Math.max(this.camera.position.distanceTo(this.controls.target), 1e-6));
    this.velocity = 0;
    this.active = false;
  }

  wheel(event) {
    if (this.disposed || this.dragging || !this.controls.enabled || !this.controls.enableZoom) return;
    const rect = this.element.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    const unit = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? this.element.clientHeight : 1;
    const delta = event.deltaY * unit;
    if (!Number.isFinite(delta) || delta === 0) return;
    event.preventDefault();
    event.stopImmediatePropagation();
    this.controls.dispatchEvent({type: 'zoom-input'});
    if (!this.active) this.cancel();
    this.pointer.x = (event.clientX - rect.left) / rect.width * 2 - 1;
    this.pointer.y = 1 - (event.clientY - rect.top) / rect.height * 2;
    const [min, max] = this.limits();
    this.goal = Math.max(Math.log(min), Math.min(Math.log(max),
      this.goal + Math.max(-150, Math.min(150, delta)) * .0025 * this.controls.zoomSpeed));
    this.active = true;
    if (this.preference?.matches) {
      this.applyDistance(Math.exp(this.goal));
      this.cancel();
      this.controls.update();
    }
  }

  update(seconds) {
    if (!this.active || this.disposed) return false;
    if (!this.controls.enabled || !this.controls.enableZoom) { this.cancel(); return false; }
    if (!Number.isFinite(seconds) || seconds <= 0) return false;
    // Exact overdamped spring solution (roots -20 and -24). Actual elapsed
    // time gives the same response on 30/60/144 Hz displays and after a pause
    [this.value, this.velocity] = stepCameraSpring(this.value, this.velocity, this.goal, seconds);
    const [min, max] = this.limits();
    const bounded = Math.max(Math.log(min), Math.min(Math.log(max), this.value));
    if (bounded !== this.value) { this.value = bounded; this.velocity = 0; }
    if (Math.abs(this.value - this.goal) < 1e-5 && Math.abs(this.velocity) < 1e-4) {
      this.value = this.goal; this.velocity = 0; this.active = false;
    }
    this.applyDistance(Math.exp(this.value));
    return true;
  }

  applyDistance(distance) {
    const [min, max] = this.limits();
    distance = Math.max(min, Math.min(max, distance));
    const previous = this.camera.position.distanceTo(this.controls.target);
    if (previous <= 0) return;
    if (this.controls.zoomToCursor) {
      // Same perspective dolly and orbit-target placement as OrbitControls,
      // spread across rendered frames so the point under the cursor stays put
      this.camera.getWorldDirection(this.direction);
      this.ray.set(this.pointer.x, this.pointer.y, 1).unproject(this.camera)
        .sub(this.camera.position).normalize();
      this.camera.position.addScaledVector(this.ray, previous - distance);
      this.controls.target.copy(this.camera.position).addScaledVector(this.direction, distance);
    } else {
      this.camera.position.sub(this.controls.target).multiplyScalar(distance / previous).add(this.controls.target);
    }
    this.camera.updateMatrixWorld();
  }

  dispose() {
    this.cancel(); this.disposed = true;
    this.element.removeEventListener('wheel', this.onWheel, true);
    this.controls.removeEventListener('start', this.onStart);
    this.controls.removeEventListener('end', this.onEnd);
    this.preference?.removeEventListener('change', this.onPreference);
  }
}
