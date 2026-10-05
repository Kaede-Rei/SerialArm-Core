import * as THREE from './vendor/three/build/three.module.js';
import {OrbitControls} from './vendor/three/jsm/controls/OrbitControls.js';
import {STLLoader} from './vendor/three/jsm/loaders/STLLoader.js';
import {GLTFLoader} from './vendor/three/jsm/loaders/GLTFLoader.js';

const decode = value => {
  const raw = atob(value);
  const bytes = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; ++i) bytes[i] = raw.charCodeAt(i);
  return bytes.buffer;
};

const euler = rpy => new THREE.Euler(rpy?.[0] || 0, rpy?.[1] || 0, rpy?.[2] || 0, 'XYZ');
const originMatrix = origin => new THREE.Matrix4().compose(
  new THREE.Vector3(...(origin?.xyz || [0, 0, 0])),
  new THREE.Quaternion().setFromEuler(euler(origin?.rpy || [0, 0, 0])),
  new THREE.Vector3(1, 1, 1),
);
const frameMatrix = frame => new THREE.Matrix4().compose(
  new THREE.Vector3(...(frame?.position || [0, 0, 0])),
  new THREE.Quaternion(...(frame?.quaternion || [0, 0, 0, 1])),
  new THREE.Vector3(1, 1, 1),
);

function jacobiEigen(matrix) {
  const a = matrix.map(row => row.slice());
  const v = [[1,0,0],[0,1,0],[0,0,1]];
  for (let iter = 0; iter < 32; ++iter) {
    let p = 0, q = 1, max = Math.abs(a[0][1]);
    for (const [i,j] of [[0,2],[1,2]]) if (Math.abs(a[i][j]) > max) { p=i; q=j; max=Math.abs(a[i][j]); }
    if (max < 1e-12) break;
    const phi = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
    const c = Math.cos(phi), s = Math.sin(phi);
    for (let k = 0; k < 3; ++k) {
      const apk = a[p][k], aqk = a[q][k];
      a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk;
    }
    for (let k = 0; k < 3; ++k) {
      const akp = a[k][p], akq = a[k][q];
      a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq;
      const vkp = v[k][p], vkq = v[k][q];
      v[k][p] = c * vkp - s * vkq; v[k][q] = s * vkp + c * vkq;
    }
  }
  return {values:[a[0][0],a[1][1],a[2][2]], vectors:v};
}

function inertiaShape(inertial) {
  if (!inertial?.valid || !(inertial.mass > 0)) return null;
  const {values, vectors} = jacobiEigen(inertial.inertia);
  const [ix, iy, iz] = values;
  const m = inertial.mass;
  const squares = [5 * (iy + iz - ix) / (2*m), 5 * (ix + iz - iy) / (2*m), 5 * (ix + iy - iz) / (2*m)];
  if (squares.some(v => !Number.isFinite(v) || v <= 0)) return null;
  const rotation = new THREE.Matrix4().set(
    vectors[0][0], vectors[0][1], vectors[0][2], 0,
    vectors[1][0], vectors[1][1], vectors[1][2], 0,
    vectors[2][0], vectors[2][1], vectors[2][2], 0,
    0, 0, 0, 1,
  );
  return {radii:squares.map(Math.sqrt), rotation};
}

function material(rgba, collision=false) {
  const c = rgba || (collision ? [0.1,0.65,1,0.25] : [0.67,0.69,0.74,1]);
  return new THREE.MeshStandardMaterial({
    color:new THREE.Color(c[0], c[1], c[2]), transparent:c[3] < 0.999 || collision,
    opacity:collision ? Math.min(c[3] ?? 1, 0.28) : c[3], wireframe:collision,
    roughness:0.68, metalness:0.08, side:THREE.DoubleSide,
  });
}

export class SerialArmModelView {
  constructor(container, resourceReader) {
    this.container = container;
    this.resourceReader = resourceReader;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(42, 1, 0.002, 100);
    this.camera.position.set(1.2, 0.9, 0.8);
    this.renderer = new THREE.WebGLRenderer({antialias:true, alpha:true, powerPreference:'high-performance'});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.setClearColor(0x000000, 0);
    Object.assign(this.renderer.domElement.style,{display:'block',width:'100%',height:'100%',maxWidth:'100%',maxHeight:'100%'});
    container.replaceChildren(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.mouseButtons.MIDDLE = THREE.MOUSE.PAN;
    this.controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
    this.controls.target.set(0,0,0.2);
    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x29313a, 2.3));
    const key = new THREE.DirectionalLight(0xffffff, 2.2); key.position.set(2,2,3); this.scene.add(key);
    this.grid = new THREE.GridHelper(2.5, 25, 0x6b7280, 0x343841); this.grid.rotateX(Math.PI/2); this.scene.add(this.grid);
    this.linkGroups = new Map(); this.frameHelpers = new Map(); this.jointAxes = []; this.labels=[]; this.effectiveGroups=[]; this.comparisonGroups=[]; this.objectVisibility={links:new Map(),linkFrames:new Map(),joints:new Map()};
    this.comGroup = new THREE.Group(); this.root.add(this.comGroup);
    this.layers = {visual:true, collision:false, linkFrames:true, jointAxes:true, com:true, inertia:true, labels:true};
    this.resizeObserver = new ResizeObserver(() => this.resize()); this.resizeObserver.observe(container);
    this.running = true; this.animate();
  }

  makeLabel(text, position=[0,0,0], layers=['labels']) {
    const canvas=document.createElement('canvas'); canvas.width=512; canvas.height=96;
    const context=canvas.getContext('2d'); context.clearRect(0,0,canvas.width,canvas.height);
    context.font='500 34px sans-serif'; const width=Math.min(context.measureText(text).width+36,500);
    context.fillStyle='rgba(12,14,20,.78)'; context.fillRect(0,8,width,64);
    context.fillStyle='#e8eaf0'; context.fillText(text,18,52);
    const texture=new THREE.CanvasTexture(canvas); texture.colorSpace=THREE.SRGBColorSpace;
    const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,transparent:true,depthTest:false}));
    sprite.position.set(...position); sprite.scale.set(Math.max(width/1450,0.085),0.045,1); sprite.userData.layers=layers.slice(); sprite.userData.texture=texture;
    this.labels.push(sprite); return sprite;
  }

  async setModel(payload) {
    this.disposeRoot();
    this.payload = payload;
    const frames = new Map((payload.native?.state?.frames || []).map(x => [x.name, x]));
    for (const link of payload.model?.links || []) {
      const group = new THREE.Group(); group.name = link.name; group.userData.kind='link'; group.userData.name=link.name; group.userData.ownerKind='links'; group.userData.ownerName=link.name; this.objectVisibility.links.set(link.name,true);
      const frame = frames.get(link.name); if (frame) group.matrix.copy(frameMatrix(frame));
      group.matrixAutoUpdate = false; this.root.add(group); this.linkGroups.set(link.name, group);
      for (const item of link.visuals || []) await this.addGeometry(group, item, false);
      for (const item of link.collisions || []) await this.addGeometry(group, item, true);
      if (link.inertial?.valid) this.addInertia(group, link.inertial);
      const linkLabel=this.makeLabel(link.name,[0,0,0.045]); linkLabel.userData.ownerKind='links'; linkLabel.userData.ownerName=link.name; group.add(linkLabel);
    }
    // Link Frame 图层只显示 URDF Link 对应的 Frame
    // Core/Pinocchio 的 state.frames 还可能包含 Joint 或内部 Frame，它们不能混入 Link Frame 图层
    for (const link of payload.model?.links || []) {
      const frame = frames.get(link.name);
      if (frame) this.addFrame(frame, payload.native?.config?.base_frame === link.name ? 0.16 : 0.08);
    }
    for (const joint of payload.model?.joints || []) this.addJointAxis(joint, frames);
    this.addEffectiveInertias(payload, frames);
    this.updateCenterOfMass(payload.native);
    this.setLayers(this.layers); this.refreshVisibility(); this.resetView();
  }

  async addGeometry(group, item, collision) {
    if (item.error || !item.geometry) return;
    const holder = new THREE.Group(); holder.userData.layer = collision ? 'collision' : 'visual'; holder.matrix.copy(originMatrix(item.origin)); holder.matrixAutoUpdate=false;
    const g = item.geometry; let object;
    if (g.type === 'box') object = new THREE.Mesh(new THREE.BoxGeometry(...g.size), material(item.rgba, collision));
    else if (g.type === 'cylinder') object = new THREE.Mesh(new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32), material(item.rgba, collision));
    else if (g.type === 'sphere') object = new THREE.Mesh(new THREE.SphereGeometry(g.radius, 32, 18), material(item.rgba, collision));
    else if (g.type === 'mesh') {
      try {
        const resource = await this.resourceReader(g.path); const data = decode(resource.data);
        if (resource.extension === '.stl') {
          const geometry = new STLLoader().parse(data); object = new THREE.Mesh(geometry, material(item.rgba, collision));
        } else {
          const gltf = await new Promise((resolve,reject)=>new GLTFLoader().parse(data, '', resolve, reject)); object = gltf.scene;
          object.traverse(node => { if (node.isMesh) { node.material = material(item.rgba, collision); } });
        }
        object.scale.set(...(g.scale || [1,1,1]));
      } catch (error) {
        holder.userData.loadError = error.message; return;
      }
    }
    if (object) { const layer=collision?'collision':'visual'; holder.userData.ownerKind='links'; holder.userData.ownerName=group.name; object.traverse?.(node=>{if(node.isMesh){node.userData.layer=layer;node.userData.ownerKind='links';node.userData.ownerName=group.name;}}); object.userData.layer=layer; holder.add(object); group.add(holder); }
  }

  addFrame(frame, size) {
    // Pinocchio 可能为同名 URDF 对象暴露多个内部 Frame
    // Link Frame 图层对每个 URDF Link 只允许存在一个 helper，避免出现无法单独关闭的重叠副本
    if (this.frameHelpers.has(frame.name)) return;
    const length=Math.max(size,0.13);
    const holder=new THREE.Group(); holder.userData.layer='linkFrames'; holder.userData.ownerKind='linkFrames'; holder.userData.ownerName=frame.name;
    holder.matrix.copy(frameMatrix(frame)); holder.matrixAutoUpdate=false;
    this.objectVisibility.linkFrames.set(frame.name,true);
    const addArrow=(direction,color)=>{
      const arrow=new THREE.ArrowHelper(direction,new THREE.Vector3(),length,color,Math.min(length*0.28,0.04),Math.min(length*0.13,0.018));
      arrow.userData.layer='linkFrames';
      arrow.traverse(node=>{if(node.material){node.material.depthTest=false;node.material.depthWrite=false;}node.renderOrder=40;});
      holder.add(arrow);
    };
    addArrow(new THREE.Vector3(1,0,0),0xef4444);
    addArrow(new THREE.Vector3(0,1,0),0x22c55e);
    addArrow(new THREE.Vector3(0,0,1),0x3b82f6);
    const outer=new THREE.Mesh(new THREE.SphereGeometry(Math.max(length*0.085,0.011),18,12),new THREE.MeshBasicMaterial({color:0x111827,depthTest:false,depthWrite:false})); outer.userData.layer='linkFrames'; outer.renderOrder=41; holder.add(outer);
    const origin=new THREE.Mesh(new THREE.SphereGeometry(Math.max(length*0.052,0.007),18,12),new THREE.MeshBasicMaterial({color:0xffffff,depthTest:false,depthWrite:false})); origin.userData.layer='linkFrames'; origin.renderOrder=42; holder.add(origin);
    this.root.add(holder); this.frameHelpers.set(frame.name,holder);
  }

  addJointAxis(joint, frames) {
    this.objectVisibility.joints.set(joint.name,true);
    const transform=this.jointTransform(joint,frames); if(!transform)return;
    const movable=['revolute','continuous','prismatic'].includes(joint.type);
    const position=new THREE.Vector3().setFromMatrixPosition(transform);
    let arrow=null;
    if(movable){
      const axis=new THREE.Vector3(...(joint.axis||[1,0,0])).normalize().transformDirection(transform);
      arrow=new THREE.ArrowHelper(axis,position,joint.controlled?0.16:0.11,joint.controlled?0xff9f1a:0x8b93a3,0.045,0.025);
      arrow.userData.layer='jointAxes'; arrow.userData.ownerKind='joints'; arrow.userData.ownerName=joint.name; this.root.add(arrow);
    }
    const color=joint.controlled?0xff9f1a:(movable?0x8b93a3:0xb0b6c2);
    const origin=new THREE.Mesh(new THREE.SphereGeometry(movable?0.008:0.006,14,10),new THREE.MeshBasicMaterial({color,depthTest:false})); origin.position.copy(position); origin.userData.layer='jointAxes'; origin.userData.ownerKind='joints'; origin.userData.ownerName=joint.name; this.root.add(origin);
    const label=this.makeLabel(joint.name,[0,0,0]); label.userData.ownerKind='joints'; label.userData.ownerName=joint.name; label.position.copy(position).add(new THREE.Vector3(0.025,0.025,0.025)); this.root.add(label); this.jointAxes.push({joint,arrow,origin,label});
  }

  jointTransform(joint,frames) {
    const parent=frames.get(joint.parent);
    if(!parent && joint.parent)return null;
    return (parent?frameMatrix(parent):new THREE.Matrix4()).multiply(originMatrix(joint.origin));
  }

  addInertia(group, inertial) {
    const xyz=inertial.origin?.xyz||[0,0,0];
    const lineGeometry=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(...xyz)]);
    const line=new THREE.Line(lineGeometry,new THREE.LineBasicMaterial({color:0xff4f87,transparent:true,opacity:0.95,depthTest:false})); line.userData.layer='com'; group.add(line);
    const local=new THREE.Group(); local.userData.inertiaKind='raw'; local.matrix.copy(originMatrix(inertial.origin)); local.matrixAutoUpdate=false;
    const inertialAxes=new THREE.AxesHelper(0.085); inertialAxes.userData.layer='inertia'; inertialAxes.userData.inertiaFrame='urdf'; local.add(inertialAxes);
    const point=new THREE.Mesh(new THREE.SphereGeometry(0.018,20,14),new THREE.MeshBasicMaterial({color:0xff4f87,depthTest:false})); point.userData.layer='com'; local.add(point);
    const halo=new THREE.Mesh(new THREE.RingGeometry(0.024,0.03,24),new THREE.MeshBasicMaterial({color:0xff4f87,side:THREE.DoubleSide,transparent:true,opacity:0.9,depthTest:false})); halo.userData.layer='com'; local.add(halo);
    const shape=inertiaShape(inertial);
    if(shape){
      const max=Math.max(...shape.radii),scale=max>0?Math.min(0.16/max,1):1;
      const ellipsoid=new THREE.Mesh(new THREE.SphereGeometry(1,24,16),new THREE.MeshBasicMaterial({color:0xff4f87,wireframe:true,transparent:true,opacity:0.72,depthTest:false}));
      ellipsoid.scale.set(shape.radii[0]*scale,shape.radii[1]*scale,shape.radii[2]*scale); ellipsoid.applyMatrix4(shape.rotation); ellipsoid.userData.layer='inertia'; local.add(ellipsoid);
      const principal=new THREE.AxesHelper(0.11); principal.applyMatrix4(shape.rotation); principal.userData.layer='inertia'; principal.userData.inertiaFrame='principal'; local.add(principal);
    }
    group.add(local);
  }

  addEffectiveInertias(payload,frames){
    const joints=new Map((payload.model?.joints||[]).map(j=>[j.name,j]));
    for(const item of payload.native?.info?.effective_inertias||[]){
      const joint=joints.get(item.joint_name); if(!joint)continue; const transform=this.jointTransform(joint,frames); if(!transform)continue;
      const holder=new THREE.Group(); holder.userData.inertiaKind='effective'; holder.userData.ownerKind='joints'; holder.userData.ownerName=joint.name; holder.matrix.copy(transform); holder.matrixAutoUpdate=false;
      const local=new THREE.Group(); local.position.set(...(item.center_of_mass||[0,0,0]));
      const point=new THREE.Mesh(new THREE.SphereGeometry(0.013,16,12),new THREE.MeshBasicMaterial({color:0xffb02e,depthTest:false})); point.userData.layer='com'; local.add(point);
      const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(...(item.center_of_mass||[0,0,0]))]),new THREE.LineBasicMaterial({color:0xffb02e,transparent:true,opacity:0.9,depthTest:false})); line.userData.layer='com'; holder.add(line);
      const shape=inertiaShape({valid:true,mass:item.mass,inertia:item.inertia});
      if(shape){const max=Math.max(...shape.radii),scale=max>0?Math.min(0.18/max,1):1;const ellipsoid=new THREE.Mesh(new THREE.SphereGeometry(1,20,14),new THREE.MeshBasicMaterial({color:0xffa928,wireframe:true,transparent:true,opacity:0.28}));ellipsoid.scale.set(shape.radii[0]*scale,shape.radii[1]*scale,shape.radii[2]*scale);ellipsoid.applyMatrix4(shape.rotation);ellipsoid.userData.layer='inertia';local.add(ellipsoid);}
      holder.add(local); this.root.add(holder); this.effectiveGroups.push({joint,holder});
    }
  }

  updateCenterOfMass(native){
    while(this.comGroup.children.length){const child=this.comGroup.children[0];this.comGroup.remove(child);child.geometry?.dispose?.();child.material?.dispose?.();child.userData?.texture?.dispose?.();}
    const com=native?.state?.center_of_mass;if(!Array.isArray(com)||com.length!==3)return;
    const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(...com)]),new THREE.LineBasicMaterial({color:0x22d3a6,transparent:true,opacity:0.95,depthTest:false}));line.userData.layer='com';this.comGroup.add(line);
    const point=new THREE.Mesh(new THREE.SphereGeometry(0.014,16,12),new THREE.MeshBasicMaterial({color:0x22d3a6,depthTest:false}));point.position.set(...com);point.userData.layer='com';this.comGroup.add(point);
    const label=this.makeLabel('Total COM',[com[0]+0.025,com[1]+0.025,com[2]+0.025],['com','labels']);this.comGroup.add(label);
    this.refreshVisibility();
  }

  updatePose(native) {
    const frames=new Map((native?.state?.frames||[]).map(x=>[x.name,x]));
    for(const [name,group] of this.linkGroups){const frame=frames.get(name);if(frame)group.matrix.copy(frameMatrix(frame));}
    for(const [name,helper] of this.frameHelpers){const frame=frames.get(name);if(frame)helper.matrix.copy(frameMatrix(frame));}
    for(const entry of this.jointAxes){const transform=this.jointTransform(entry.joint,frames);if(!transform)continue;const position=new THREE.Vector3().setFromMatrixPosition(transform);if(entry.arrow){entry.arrow.position.copy(position);entry.arrow.setDirection(new THREE.Vector3(...(entry.joint.axis||[1,0,0])).normalize().transformDirection(transform));}entry.origin.position.copy(position);entry.label.position.copy(position).add(new THREE.Vector3(0.02,0.02,0.02));}
    for(const entry of this.effectiveGroups){const transform=this.jointTransform(entry.joint,frames);if(transform)entry.holder.matrix.copy(transform);}
    this.updateCenterOfMass(native);
  }

  clearGravityComparison() {
    for (const group of this.comparisonGroups) {
      group.parent?.remove(group);
      group.traverse(node=>{node.geometry?.dispose?.();if(node.material){const list=Array.isArray(node.material)?node.material:[node.material];for(const material of list)material.dispose?.();}node.userData?.texture?.dispose?.();});
    }
    this.comparisonGroups=[];
  }

  setGravityComparison(result) {
    this.clearGravityComparison();
    if(!result?.first_moments?.length||!this.payload)return;
    const links=new Map((this.payload.model?.links||[]).map(link=>[link.name,link]));
    for(const item of result.first_moments){
      const link=links.get(item.link_name),holder=this.linkGroups.get(item.link_name);
      const mass=Number(link?.inertial?.mass||0);
      if(!holder||!(mass>0)||!Array.isArray(item.value)||item.value.length!==3)continue;
      const original=(link.inertial?.origin?.xyz||[0,0,0]).map(Number);
      const candidate=item.value.map(value=>Number(value)/mass);
      if(candidate.some(value=>!Number.isFinite(value)))continue;
      const group=new THREE.Group();group.userData.layer='candidate';
      const originalPoint=new THREE.Mesh(new THREE.SphereGeometry(0.012,18,12),new THREE.MeshBasicMaterial({color:0xff4f87,depthTest:false,depthWrite:false}));originalPoint.position.set(...original);originalPoint.renderOrder=60;group.add(originalPoint);
      const candidatePoint=new THREE.Mesh(new THREE.SphereGeometry(0.015,18,12),new THREE.MeshBasicMaterial({color:0x22c7e8,depthTest:false,depthWrite:false}));candidatePoint.position.set(...candidate);candidatePoint.renderOrder=61;group.add(candidatePoint);
      const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...original),new THREE.Vector3(...candidate)]),new THREE.LineBasicMaterial({color:0x22c7e8,transparent:true,opacity:0.95,depthTest:false,depthWrite:false}));line.renderOrder=60;group.add(line);
      const label=this.makeLabel(`${item.link_name} candidate`,[candidate[0]+0.018,candidate[1]+0.018,candidate[2]+0.018],['candidate']);label.scale.multiplyScalar(0.82);group.add(label);
      holder.add(group);this.comparisonGroups.push(group);
    }
    this.refreshVisibility();
  }

  setLayers(layers) {
    this.layers = {...this.layers, ...layers};
    this.refreshVisibility();
  }

  setObjectVisible(kind,name,visible) {
    if(kind==='linkFrames') { this.setLinkFrameVisible(name,visible); return; }
    if(!this.objectVisibility[kind])return;
    this.objectVisibility[kind].set(name,visible!==false);
    this.refreshVisibility();
  }

  setLinkFrameVisible(name,visible) {
    const next=visible!==false;
    this.objectVisibility.linkFrames.set(name,next);
    const helper=this.frameHelpers.get(name);
    if(helper)helper.visible=!!this.layers.linkFrames&&next;
  }

  refreshVisibility() {
    this.root.traverse(node => {
      const requiredLayers=Array.isArray(node.userData?.layers) ? node.userData.layers : (node.userData?.layer ? [node.userData.layer] : []);
      let visible=requiredLayers.every(layer => !(layer in this.layers) || !!this.layers[layer]);
      const kind=node.userData?.ownerKind,name=node.userData?.ownerName;
      if(kind&&name&&this.objectVisibility[kind])visible=visible&&this.objectVisibility[kind].get(name)!==false;
      node.visible=visible;
    });
    for(const [name,helper] of this.frameHelpers)helper.visible=!!this.layers.linkFrames&&this.objectVisibility.linkFrames.get(name)!==false;
  }

  select(name) {
    this.selection=name||'';
    const selectedJoint=(this.payload?.model?.joints||[]).find(j=>j.name===name);
    const selectedLinks=new Set(selectedJoint?[selectedJoint.parent,selectedJoint.child]:[name]);
    for(const [linkName,group] of this.linkGroups){
      const selected=selectedLinks.has(linkName);
      group.traverse(node=>{
        if(!node.isMesh||!node.material||!['visual','collision'].includes(node.userData?.layer))return;
        const materials=Array.isArray(node.material)?node.material:[node.material];
        for(const m of materials){
          if(m.userData.baseOpacity===undefined)m.userData.baseOpacity=m.opacity;
          if(m.userData.baseTransparent===undefined)m.userData.baseTransparent=m.transparent;
          if('emissive' in m&&m.userData.baseEmissive===undefined)m.userData.baseEmissive=m.emissive.getHex();
          if(name){m.transparent=true;m.opacity=selected?m.userData.baseOpacity:Math.min(m.userData.baseOpacity,0.22);}else{m.transparent=m.userData.baseTransparent;m.opacity=m.userData.baseOpacity;}
          if('emissive' in m)m.emissive.setHex(selected?0x5a3200:m.userData.baseEmissive||0);
          m.needsUpdate=true;
        }
      });
    }
    for(const entry of this.jointAxes){const selected=entry.joint.name===name;if(entry.arrow){entry.arrow.setColor(new THREE.Color(selected?0xffd166:(entry.joint.controlled?0xff9f1a:0x8b93a3)));entry.arrow.setLength(selected?0.21:(entry.joint.controlled?0.16:0.11),selected?0.055:0.045,selected?0.032:0.025);}entry.origin.scale.setScalar(selected?1.6:1);}
    this.refreshVisibility();
  }

  resetView() {
    this.root.updateMatrixWorld(true);
    const box=new THREE.Box3();
    this.root.traverse(node=>{if(!node.isMesh||node.userData?.layer!=='visual'||!node.geometry)return;node.geometry.computeBoundingBox?.();if(node.geometry.boundingBox)box.union(node.geometry.boundingBox.clone().applyMatrix4(node.matrixWorld));});
    if(box.isEmpty())box.setFromObject(this.root);
    const size=box.getSize(new THREE.Vector3()); const center=box.getCenter(new THREE.Vector3());
    const radius=Math.max(size.length()*0.62,0.35); this.controls.target.copy(center); this.camera.position.copy(center).add(new THREE.Vector3(radius*1.35,radius*1.05,radius*0.95));
    this.camera.near=Math.max(radius/1000,0.001); this.camera.far=Math.max(radius*50,20); this.camera.updateProjectionMatrix(); this.controls.update();
  }

  resize() {
    const width=Math.max(this.container.clientWidth,1), height=Math.max(this.container.clientHeight,1); this.renderer.setSize(width,height,false); this.camera.aspect=width/height; this.camera.updateProjectionMatrix();
  }

  animate() { if (!this.running) return; requestAnimationFrame(()=>this.animate()); this.controls.update(); this.renderer.render(this.scene,this.camera); }

  disposeRoot() {
    this.root.traverse(node=>{if(node.geometry)node.geometry.dispose?.();if(node.material){const list=Array.isArray(node.material)?node.material:[node.material];for(const m of list)m.dispose?.();}node.userData?.texture?.dispose?.();});
    this.scene.remove(this.root); this.root=new THREE.Group(); this.scene.add(this.root); this.linkGroups.clear(); this.frameHelpers.clear(); this.jointAxes=[]; this.effectiveGroups=[]; this.labels=[]; this.comparisonGroups=[]; this.objectVisibility={links:new Map(),linkFrames:new Map(),joints:new Map()}; this.comGroup=new THREE.Group(); this.root.add(this.comGroup);
  }

  dispose() { this.running=false; this.resizeObserver.disconnect(); this.disposeRoot(); this.controls.dispose(); this.renderer.dispose(); }
}
