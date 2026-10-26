import test from 'node:test';
import assert from 'node:assert/strict';
import {PerspectiveCamera, Vector3, EventDispatcher} from 'three';
import {SmoothModelZoom} from '../renderer/model-camera-zoom.mjs';

function fixture(reduced = false) {
  class Element extends EventTarget {
    clientHeight = 400;
    getBoundingClientRect() { return {left: 20, top: 30, width: 600, height: 400}; }
  }
  const element = new Element();
  const preference = new EventTarget(); preference.matches = reduced;
  const controls = new EventDispatcher();
  Object.assign(controls, {domElement: element, object: new PerspectiveCamera(42, 1.5, .001, 100),
    target: new Vector3(0, 0, .3), enabled: true, enableZoom: true, zoomSpeed: 1,
    zoomToCursor: true, minDistance: .25, maxDistance: 10});
  controls.object.up.set(0, 0, 1);
  controls.object.position.set(1, -2, 1.5);
  controls.update = () => { controls.object.lookAt(controls.target); controls.object.updateMatrixWorld(); };
  controls.update();
  const zoom = new SmoothModelZoom(controls, preference);
  const distance = () => controls.object.position.distanceTo(controls.target);
  function wheel(deltaY, {deltaMode = 0, x = 320, y = 230, ctrlKey = false} = {}) {
    const event = new Event('wheel', {cancelable: true});
    Object.assign(event, {deltaY, deltaMode, clientX: x, clientY: y, ctrlKey});
    element.dispatchEvent(event); return event;
  }
  function advance(seconds, fps = 60) {
    for (let i = 0; i < Math.round(seconds * fps); i++) { controls.update(); zoom.update(1 / fps); }
  }
  return {zoom, controls, element, preference, wheel, distance, advance};
}
function close(actual, expected, epsilon = 1e-8) {
  assert.ok(Math.abs(actual - expected) < epsilon, `${actual} != ${expected}`);
}

test('wheel input is consumed once and zoom is spread over rendered frames', () => {
  const f = fixture(), initial = f.distance(); let stockCalls = 0;
  f.element.addEventListener('wheel', () => stockCalls++);
  assert.equal(f.wheel(100).defaultPrevented, true);
  close(f.distance(), initial); assert.equal(stockCalls, 0);
  f.advance(1 / 60);
  assert.ok(f.distance() > initial && f.distance() < initial * Math.exp(.25));
  const first = f.distance(); f.advance(1 / 60); assert.ok(f.distance() > first);
  f.advance(1); close(f.distance(), initial * Math.exp(.25)); assert.equal(f.zoom.active, false);
});

test('30, 60 and 144 Hz converge to the same camera distance at the same time', () => {
  const results = [30, 60, 144].map(fps => { const f = fixture(); f.wheel(-100); f.advance(.5, fps); return f.distance(); });
  for (const result of results) close(result, results[0]);
});

test('successive wheel events accumulate without jumping or resetting the current motion', () => {
  const f = fixture(), initial = f.distance();
  f.wheel(-48); f.advance(.05);
  const shown = f.distance(); f.wheel(-48); close(f.distance(), shown);
  f.advance(1); close(f.distance(), initial * Math.exp(-96 * .0025));
});

test('reversing the wheel changes direction smoothly and settles at the new target', () => {
  const f = fixture(), initial = f.distance(); f.wheel(100); f.advance(.1);
  const shown = f.distance(); f.wheel(-150); close(f.distance(), shown);
  f.advance(1); close(f.distance(), initial * Math.exp(-50 * .0025));
});

test('pixel, line and page deltas use the reference canvas normalization', () => {
  const results = [[48, 0], [3, 1], [.12, 2]].map(([delta, deltaMode]) => {
    const f = fixture(); f.wheel(delta, {deltaMode}); f.advance(1); return f.distance();
  });
  for (const result of results) close(result, results[0]);
});

test('fine trackpad and pinch input remains proportional without the stock Ctrl multiplier', () => {
  const f = fixture(), initial = f.distance(); f.wheel(.25, {ctrlKey: true}); f.advance(1);
  close(f.distance(), initial * Math.exp(.25 * .0025));
});

test('large wheel deltas are bounded per event', () => {
  const f = fixture(), initial = f.distance(); f.wheel(100000); f.advance(1);
  close(f.distance(), initial * Math.exp(150 * .0025));
});

test('the world point under an off-center cursor stays at the same screen position', () => {
  const f = fixture(), camera = f.controls.object;
  const point = new Vector3(.4, -.25, 1).unproject(camera).sub(camera.position).normalize()
    .multiplyScalar(2).add(camera.position);
  const before = point.clone().project(camera);
  f.wheel(-100, {x: 440, y: 280});
  for (let i = 0; i < 45; i++) {
    f.controls.update(); f.zoom.update(1 / 60);
    const shown = point.clone().project(camera); close(shown.x, before.x); close(shown.y, before.y);
  }
});

test('distance limits hold during motion and do not accumulate hidden overscroll', () => {
  const f = fixture();
  for (let i = 0; i < 100; i++) f.wheel(150);
  f.advance(1); close(f.distance(), f.controls.maxDistance);
  f.wheel(-100); f.advance(1); close(f.distance(), 10 * Math.exp(-.25));
  for (let i = 0; i < 100; i++) f.wheel(-150);
  for (let i = 0; i < 90; i++) {
    f.controls.update(); f.zoom.update(1 / 60);
    assert.ok(f.distance() >= f.controls.minDistance - 1e-10 && f.distance() <= f.controls.maxDistance + 1e-10);
  }
  close(f.distance(), .25);
  f.wheel(100); f.advance(1); close(f.distance(), .25 * Math.exp(.25));
});

test('manual orbit or touch takes over the shown camera without a zoom tail', () => {
  const f = fixture(); f.wheel(-100); f.advance(.1);
  f.controls.dispatchEvent({type: 'start'}); const shown = f.distance();
  f.advance(1); close(f.distance(), shown);
  assert.equal(f.wheel(-100).defaultPrevented, false);
  f.controls.dispatchEvent({type: 'end'}); f.wheel(-100); f.advance(.5);
  assert.ok(f.distance() < shown);
});

test('resetting or resizing can cancel pending zoom without later drift', () => {
  const f = fixture(); f.wheel(-100); f.advance(.05); f.zoom.cancel();
  f.controls.object.position.set(2, -3, 2); f.controls.update();
  const reset = f.distance(); f.advance(1); close(f.distance(), reset);
});

test('disabled controls ignore wheel input and cancel an active zoom', () => {
  const f = fixture(); f.controls.enabled = false;
  assert.equal(f.wheel(-100).defaultPrevented, false);
  f.controls.enabled = true; f.wheel(-100); f.advance(.05); f.controls.enableZoom = false;
  const shown = f.distance(); f.advance(1); close(f.distance(), shown); assert.equal(f.zoom.active, false);
});

test('reduced motion applies the target immediately and reacts to preference changes', () => {
  const f = fixture(true), initial = f.distance(); f.wheel(-100);
  close(f.distance(), initial * Math.exp(-.25)); assert.equal(f.zoom.active, false);
  f.preference.matches = false; f.wheel(-100); f.advance(.05);
  f.preference.matches = true; f.preference.dispatchEvent(new Event('change'));
  close(f.distance(), initial * Math.exp(-.5)); assert.equal(f.zoom.active, false);
});

test('center zoom keeps the orbit target fixed when cursor zoom is disabled', () => {
  const f = fixture(); f.controls.zoomToCursor = false; const target = f.controls.target.clone();
  f.wheel(-100, {x: 440, y: 280}); f.advance(1);
  close(f.controls.target.distanceTo(target), 0);
});

test('a background pause settles safely and disposal releases the wheel listener', () => {
  const f = fixture(), initial = f.distance(); f.wheel(-100); f.zoom.update(20);
  close(f.distance(), initial * Math.exp(-.25));
  f.zoom.dispose(); const shown = f.distance();
  assert.equal(f.wheel(100).defaultPrevented, false); f.advance(1); close(f.distance(), shown);
});
