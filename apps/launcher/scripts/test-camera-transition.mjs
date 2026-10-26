import test from 'node:test';
import assert from 'node:assert/strict';
import {PerspectiveCamera, Vector3, EventDispatcher} from 'three';
import {ModelCameraTransition} from '../renderer/model-camera-transition.mjs';
import {SmoothModelZoom} from '../renderer/model-camera-zoom.mjs';

const views = {front:[0,-1,0],back:[0,1,0],left:[-1,0,0],right:[1,0,0],top:[0,-.001,1],bottom:[0,.001,-1],isometric:[1.25,-1.35,.95]};
function close(actual, expected, epsilon=1e-7) { assert.ok(Math.abs(actual-expected)<epsilon, `${actual} != ${expected}`); }
function same(actual, expected, epsilon=1e-7) { close(actual.distanceTo(expected),0,epsilon); }
function fixture(reduced=false) {
  const preference = new EventTarget(); preference.matches=reduced;
  const element = new EventTarget();
  element.clientHeight=400; element.getBoundingClientRect=()=>({left:0,top:0,width:600,height:400});
  const camera=new PerspectiveCamera(42,1.5,.001,100); camera.up.set(0,0,1);
  const controls=new EventDispatcher(); Object.assign(controls,{object:camera,domElement:element,
    target:new Vector3(1,2,.5),minDistance:.1,maxDistance:20,minPolarAngle:.001,maxPolarAngle:Math.PI-.001,
    enabled:true,enableZoom:true,enableDamping:true,zoomSpeed:1,zoomToCursor:true});
  camera.position.copy(controls.target).add(new Vector3(2,-3,1.5));
  controls.update=()=>{camera.lookAt(controls.target);camera.updateMatrixWorld();};controls.update();
  const zoom=new SmoothModelZoom(controls,preference), motion=new ModelCameraTransition(controls,preference);
  const distance=()=>camera.position.distanceTo(controls.target);
  const direction=()=>camera.position.clone().sub(controls.target).normalize();
  function move(view,radius=distance(),center=controls.target,immediate=false) {
    zoom.cancel(); motion.move(new Vector3(...(views[view]||view)),radius,center,immediate);
  }
  function step(seconds) { controls.update(); motion.update(seconds); zoom.update(seconds); }
  function advance(seconds,fps=60) { for(let i=0;i<Math.round(seconds*fps);i++)step(1/fps); }
  function wheel(deltaY) { const event=new Event('wheel',{cancelable:true});Object.assign(event,{deltaY,deltaMode:0,clientX:300,clientY:200});element.dispatchEvent(event); }
  return {preference,controls,camera,zoom,motion,distance,direction,move,step,advance,wheel};
}

test('a view command leaves the current pose intact, then moves over multiple frames',()=>{
  const f=fixture(), position=f.camera.position.clone(), target=f.controls.target.clone();
  const rotation=f.camera.quaternion.clone(), radius=f.distance();f.move('front');
  same(f.camera.position,position);same(f.controls.target,target);close(f.camera.quaternion.angleTo(rotation),0);
  f.step(1/60);assert.ok(f.camera.position.distanceTo(position)>1e-3);assert.ok(f.direction().distanceTo(new Vector3(...views.front))>1e-3);
  f.advance(1);same(f.direction(),new Vector3(...views.front));close(f.distance(),radius);assert.equal(f.motion.active,false);
});

test('every named view settles with a valid orientation and the same orbit distance',()=>{
  const f=fixture(), radius=f.distance();
  for(const [view,axis] of Object.entries(views)) {
    f.move(view);f.advance(1);
    same(f.direction(),new Vector3(...axis).normalize());close(f.distance(),radius);
    assert.ok(f.camera.quaternion.toArray().every(Number.isFinite));assert.equal(f.motion.active,false);
  }
});

test('opposite side views orbit around the robot without collapsing the camera radius',()=>{
  const f=fixture();f.move('left',3,undefined,true);f.move('right');
  for(let i=0;i<60;i++){f.step(1/60);close(f.distance(),3);close(f.direction().z,0);}
  same(f.direction(),new Vector3(...views.right));
});

test('azimuth wrapping takes the short path rather than spinning almost a full turn',()=>{
  const f=fixture();f.move([-1,-.02,0],3,undefined,true);f.move([-1,.02,0]);
  let previous=f.direction(),travel=0;
  for(let i=0;i<60;i++){f.step(1/60);const next=f.direction();travel+=previous.angleTo(next);previous=next;}
  assert.ok(travel<.05,`Unexpected rotation: ${travel}`);
});

test('fit animates the orbit center and distance together without an initial jump',()=>{
  const f=fixture(), target=f.controls.target.clone(), position=f.camera.position.clone(), center=new Vector3(-2,3,1);
  f.move('isometric',6,center);same(f.controls.target,target);same(f.camera.position,position);
  f.step(1/60);assert.ok(f.controls.target.distanceTo(target)>0);assert.ok(f.controls.target.distanceTo(center)>1);
  f.advance(1);same(f.controls.target,center);close(f.distance(),6);same(f.direction(),new Vector3(...views.isometric).normalize());
});

test('30, 60 and 144 Hz reach the same pose after the same elapsed time',()=>{
  const positions=[30,60,144].map(fps=>{const f=fixture();f.move('top',6,new Vector3(-2,3,1));f.advance(.5,fps);return f.camera.position;});
  for(const position of positions)same(position,positions[0]);
});

test('retargeting from the displayed pose preserves motion and settles at the latest request',()=>{
  const f=fixture();f.move('back');f.advance(.1);const before=f.camera.position.clone();
  f.step(.0001);const current=f.camera.position.clone(),velocity=current.clone().sub(before).divideScalar(.0001);
  f.move('left');same(f.camera.position,current);f.step(.0001);
  const after=f.camera.position.clone().sub(current).divideScalar(.0001);
  assert.ok(after.distanceTo(velocity)<velocity.length()*.02);
  f.advance(1);same(f.direction(),new Vector3(...views.left));
});

test('manual orbit takes over the displayed pose and prevents later automatic drift',()=>{
  const f=fixture();f.move('back');f.advance(.1);f.controls.dispatchEvent({type:'start'});
  const shown=f.camera.position.clone();f.advance(1);same(f.camera.position,shown);assert.equal(f.motion.active,false);
});

test('wheel zoom interrupts a view transition from the current displayed distance',()=>{
  const f=fixture();f.move('top');f.advance(.1);const shown=f.camera.position.clone(),radius=f.distance();
  f.wheel(-100);same(f.camera.position,shown);assert.equal(f.motion.active,false);assert.equal(f.zoom.active,true);
  const direction=f.direction();f.advance(1);same(f.direction(),direction);close(f.distance(),radius*Math.exp(-.25));
});

test('a view command interrupts wheel zoom without inheriting the previous zoom tail',()=>{
  const f=fixture();f.wheel(-100);f.advance(.1);const radius=f.distance();f.move('right');f.advance(1);
  close(f.distance(),radius);same(f.direction(),new Vector3(...views.right));assert.equal(f.zoom.active,false);
});

test('zero or disabled wheel input does not interrupt a running transition',()=>{
  const f=fixture();f.move('top');f.wheel(0);assert.equal(f.motion.active,true);
  f.controls.enableZoom=false;f.wheel(-100);assert.equal(f.motion.active,true);
});

test('reduced motion and an explicit initial fit apply the requested pose immediately',()=>{
  for(const reduced of [true,false]) {
    const f=fixture(reduced),center=new Vector3(-2,1,3);f.move('front',5,center,!reduced);
    same(f.controls.target,center);close(f.distance(),5);same(f.direction(),new Vector3(...views.front));assert.equal(f.motion.active,false);
  }
});

test('enabling reduced motion mid-transition immediately completes the current view',()=>{
  const f=fixture();f.move('right');f.advance(.1);f.preference.matches=true;f.preference.dispatchEvent(new Event('change'));
  same(f.direction(),new Vector3(...views.right));assert.equal(f.motion.active,false);
});

test('polar constraints settle cleanly without trying to animate past the pole',()=>{
  const f=fixture();f.controls.minPolarAngle=.2;f.move('top');f.advance(1);
  close(Math.acos(f.direction().z),.2);assert.equal(f.motion.active,false);
});

test('disabled controls stop a view transition at the displayed pose',()=>{
  const f=fixture();f.move('top');f.advance(.1);f.controls.enabled=false;const shown=f.camera.position.clone();
  f.advance(1);same(f.camera.position,shown);assert.equal(f.motion.active,false);
});

test('disposal cancels transitions and removes input and preference listeners',()=>{
  const f=fixture();f.move('top');f.advance(.1);f.motion.dispose();const shown=f.camera.position.clone();
  f.preference.matches=true;f.preference.dispatchEvent(new Event('change'));f.move('back');f.advance(1);
  same(f.camera.position,shown);assert.equal(f.motion.active,false);
});
