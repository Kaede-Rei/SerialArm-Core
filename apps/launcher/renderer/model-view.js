import * as THREE from './vendor/three/build/three.module.js';
import {OrbitControls} from './vendor/three/jsm/controls/OrbitControls.js';
import {GLTFLoader} from './vendor/three/jsm/loaders/GLTFLoader.js';
import {SmoothModelZoom} from './model-camera-zoom.mjs';
import {ModelCameraTransition} from './model-camera-transition.mjs';
import {gravityComMarkers} from './gravity-com-geometry.mjs';

const decode = value => {
  const raw = atob(value);
  const bytes = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; ++i) bytes[i] = raw.charCodeAt(i);
  return bytes.buffer;
};

// URDF RPY is a fixed-axis rotation Rz(yaw) * Ry(pitch) * Rx(roll)
const originQuaternion = rpy => {
  const [roll, pitch, yaw] = rpy || [0, 0, 0];
  return new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw)
    .multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), pitch))
    .multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), roll));
};
const originMatrix = origin => new THREE.Matrix4().compose(
  new THREE.Vector3(...(origin?.xyz || [0, 0, 0])),
  originQuaternion(origin?.rpy),
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

function disposeObject(object, {geometry = true, textures = true, materials:disposeMaterials = true} = {}) {
  const geometries = new Set(), materials = new Set(), images = new Set();
  object?.traverse(node => {
    if (geometry && node.geometry) geometries.add(node.geometry);
    for (const item of (Array.isArray(node.material) ? node.material : [node.material])) {
      if (!item) continue;
      materials.add(item);
      if (textures) for (const value of Object.values(item)) if (value?.isTexture) images.add(value);
    }
    if (textures && node.userData?.texture) images.add(node.userData.texture);
  });
  for (const value of geometries) value.dispose();
  if(disposeMaterials)for (const value of materials) value.dispose();
  for (const value of images) value.dispose();
}

function contextDiagnostics(gl) {
  const debug = gl.getExtension('WEBGL_debug_renderer_info');
  return {
    webgl2: typeof WebGL2RenderingContext !== 'undefined' && gl instanceof WebGL2RenderingContext,
    vendor: gl.getParameter(gl.VENDOR), renderer: gl.getParameter(gl.RENDERER),
    unmaskedVendor: debug ? gl.getParameter(debug.UNMASKED_VENDOR_WEBGL) : null,
    unmaskedRenderer: debug ? gl.getParameter(debug.UNMASKED_RENDERER_WEBGL) : null,
    version: gl.getParameter(gl.VERSION), shadingLanguageVersion: gl.getParameter(gl.SHADING_LANGUAGE_VERSION),
    attributes: gl.getContextAttributes(), maxTextureSize: gl.getParameter(gl.MAX_TEXTURE_SIZE),
  };
}

export class SerialArmModelView {
  constructor(container, resourceReader, language = 'zh', {mode = 'normal', onStatus} = {}) {
    this.container = container;
    this.resourceReader = resourceReader;
    this.language = language;
    this.mode = mode === 'compatibility' ? 'compatibility' : 'normal';
    this.onStatus = onStatus;
    this.disposed = false; this.running = true; this.paused = false; this.needsRender = true;
    this.modelGeneration = 0; this.drawCount = 0; this.modelLoaded = false;
    this.modelStatus = 'empty'; this.modelErrors = []; this.modelError = null;
    this.ghostLinkGroups = new Map();
    this.meshGeometryCache = new Map(); this.ghostEdges = new Map();
    this.geometryJobs = new Map(); this.geometryJobSerial = 0;
    this.poseFrames = null; this.totalComVisual = null;
    this.status = 'initializing'; this.error = null; this.suspended = false;
    this.diagnostics = {threeRevision: THREE.REVISION, mode: this.mode, creationErrors: []};
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(42, 1, 0.002, 100);
    // URDF and the robot workspace use Z-up; OrbitControls defaults to Y-up
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.2, -0.9, 0.8);
    this.canvas = document.createElement('canvas');
    this.onCreationError = event => this.diagnostics.creationErrors.push(String(event.statusMessage || 'WebGL context creation failed'));
    this.onContextLost = event => {
      event.preventDefault();
      if (!this.disposed) this.failGraphics(new Error(event.statusMessage || 'WebGL context lost'), 'context', 'lost');
    };
    this.canvas.addEventListener('webglcontextcreationerror', this.onCreationError);
    this.canvas.addEventListener('webglcontextlost', this.onContextLost);
    try {
      const attributes = {antialias: this.mode !== 'compatibility', alpha: true};
      this.diagnostics.requestedAttributes = attributes;
      // One explicit WebGL2 request records the browser's creation error and avoids
      // Three.js trying a second context with different attributes after failure
      this.gl = this.canvas.getContext('webgl2', attributes);
      if (!this.gl) throw new Error('WebGL2 context creation failed');
      Object.assign(this.diagnostics, contextDiagnostics(this.gl));
      this.renderer = new THREE.WebGLRenderer({canvas: this.canvas, context: this.gl, ...attributes});
      this.renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, this.mode === 'compatibility' ? 1 : 2));
      this.renderer.outputColorSpace = THREE.SRGBColorSpace;
      this.renderer.setClearColor(0x000000, 0);
      Object.assign(this.renderer.domElement.style,{display:'block',width:'100%',height:'100%',maxWidth:'100%',maxHeight:'100%'});
      container.replaceChildren(this.renderer.domElement);
      this.controls = new OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.09;
      this.controls.enableRotate = true;
      this.controls.enablePan = true;
      this.controls.enableZoom = true;
      this.controls.screenSpacePanning = true;
      this.controls.zoomToCursor = true;
      this.controls.minPolarAngle = 0.001;
      this.controls.maxPolarAngle = Math.PI - 0.001;
      this.controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
      this.controls.mouseButtons.MIDDLE = THREE.MOUSE.PAN;
      this.controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
      this.controls.target.set(0,0,0.2);
      this.smoothZoom = new SmoothModelZoom(this.controls);
      this.cameraTransition = new ModelCameraTransition(this.controls);
      this.installCameraTools();
      this.root = new THREE.Group();
      this.scene.add(this.root);
      this.scene.add(new THREE.HemisphereLight(0xffffff, 0x29313a, 2.3));
      const key = new THREE.DirectionalLight(0xffffff, 2.2); key.position.set(2,2,3); this.scene.add(key);
      this.grid = new THREE.GridHelper(2.5, 25, 0x6b7280, 0x343841); this.grid.rotateX(Math.PI/2); this.scene.add(this.grid);
      this.linkGroups = new Map(); this.frameHelpers = new Map(); this.jointAxes = []; this.labels=[]; this.effectiveGroups=[]; this.comparisonGroups=[]; this.objectVisibility={links:new Map(),linkFrames:new Map(),joints:new Map()};
      this.comGroup = new THREE.Group(); this.root.add(this.comGroup);
      this.layers = {visual:true, collision:false, linkFrames:false, jointAxes:true, com:false, inertia:false, labels:false};
      this.inertiaSource = 'urdf';
      this.resizeObserver = new ResizeObserver(() => this.resize()); this.resizeObserver.observe(container);
      this.onControlsChange = () => this.requestRender();
      this.controls.addEventListener('change', this.onControlsChange);
      this.controls.addEventListener('start', this.onControlsChange);
      this.controls.addEventListener('end', this.onControlsChange);
      this.controls.addEventListener('zoom-input', this.onControlsChange);
      this.onVisibilityChange = () => {
        if (document.hidden) this.stopScheduledDraw();
        else this.lastFrameTime = undefined;
        this.requestRender();
      };
      document.addEventListener('visibilitychange', this.onVisibilityChange);
      if (globalThis.IntersectionObserver) {
        this.intersectionObserver = new IntersectionObserver(entries => {
          this.offscreen = !entries[0]?.isIntersecting;
          if (this.offscreen) this.stopScheduledDraw();
          this.requestRender();
        });
        this.intersectionObserver.observe(container);
      }
      this.resize();
    } catch (cause) {
      const failure = new Error(cause?.message || String(cause), {cause});
      failure.name = 'GraphicsError'; failure.category = 'graphics'; failure.code = 'GRAPHICS_INITIALIZATION_FAILED';
      this.failGraphics(failure, 'initialization');
      failure.diagnostics = structuredClone(this.diagnostics);
      this.releaseResources();
      throw failure;
    }
  }

  getRenderState() {
    let meshCount = 0;
    this.root?.traverse(node => { if (node.isMesh && node.userData.layer === 'visual') meshCount++; });
    return {status: this.status, mode: this.mode, drawCount: this.drawCount, modelLoaded: this.modelLoaded,
      modelStatus: this.modelStatus, modelError: this.modelError ? {...this.modelError} : null,
      modelErrors: this.modelErrors.map(error => ({...error})),
      meshCount, renderCalls: this.renderer?.info.render.calls || 0, triangles: this.renderer?.info.render.triangles || 0,
      suspended: this.suspended, paused: this.paused, error: this.error ? {...this.error} : null,
      diagnostics: structuredClone(this.diagnostics)};
  }

  emitStatus() {
    try { this.onStatus?.(this.getRenderState()); }
    catch (error) { console.error('Model view status handler failed', error); }
  }

  pause() {
    if (this.disposed) return;
    this.paused = true; this.suspended = true;
    this.stopScheduledDraw();
    if (this.controls) this.controls.enabled = false;
    this.smoothZoom?.cancel(); this.cameraTransition?.cancel();
  }

  attach(container, language = this.language) {
    if (this.disposed || !container) return false;
    this.pause();
    this.resizeObserver?.disconnect(); this.intersectionObserver?.disconnect();
    this.container = container; this.offscreen = false;
    if (language !== this.language) {
      this.language = language; this.cameraTools?.remove(); this.cameraHint?.remove();
      this.installCameraTools();
    }
    container.replaceChildren(this.canvas, this.cameraTools, this.cameraHint);
    this.resizeObserver?.observe(container); this.intersectionObserver?.observe(container);
    return true;
  }

  resume() {
    if (this.disposed || !this.running) return false;
    this.paused = false; this.lastFrameTime = undefined;
    if (this.controls) this.controls.enabled = true;
    this.resize(); this.requestRender();
    return true;
  }

  failGraphics(error, phase = 'render', status = 'failed') {
    if (this.disposed || ['failed', 'lost'].includes(this.status)) return;
    this.status = status; this.running = false;
    this.error = {name: error?.name || 'GraphicsError', message: error?.message || String(error),
      category: 'graphics', code: phase === 'initialization' ? 'GRAPHICS_INITIALIZATION_FAILED' : status === 'lost' ? 'GRAPHICS_CONTEXT_LOST' : 'GRAPHICS_RENDER_FAILED', phase};
    this.stopScheduledDraw();
    if (this.controls) this.controls.enabled = false;
    this.smoothZoom?.cancel(); this.cameraTransition?.cancel();
    this.emitStatus();
  }

  captureViewState() {
    if (!this.camera || !this.controls) return null;
    return {camera: {position: this.camera.position.toArray(), quaternion: this.camera.quaternion.toArray(),
      up: this.camera.up.toArray(), near: this.camera.near, far: this.camera.far, fov: this.camera.fov, zoom: this.camera.zoom},
      target: this.controls.target.toArray(), minDistance: this.controls.minDistance, maxDistance: this.controls.maxDistance,
      layers: {...this.layers}, objectVisibility: Object.fromEntries(Object.entries(this.objectVisibility).map(([key, values]) => [key, [...values]])),
      inertiaSource: this.inertiaSource, selection: this.selection || '', hasFramedModel: !!this.hasFramedModel};
  }

  restoreViewState(state) {
    if (!state || this.disposed || !this.controls) return;
    const vector = (value, size) => Array.isArray(value) && value.length === size && value.every(Number.isFinite);
    this.smoothZoom.cancel(); this.cameraTransition.cancel();
    const damping = this.controls.enableDamping;
    this.controls.enableDamping = false; this.controls.update(); this.controls.enableDamping = damping;
    if (vector(state.camera?.position, 3)) this.camera.position.fromArray(state.camera.position);
    if (vector(state.camera?.up, 3)) this.camera.up.fromArray(state.camera.up);
    if (vector(state.target, 3)) this.controls.target.fromArray(state.target);
    for (const key of ['near', 'far', 'fov', 'zoom']) if (Number.isFinite(state.camera?.[key]) && state.camera[key] > 0) this.camera[key] = state.camera[key];
    for (const key of ['minDistance', 'maxDistance']) if (Number.isFinite(state[key]) && state[key] > 0) this.controls[key] = state[key];
    this.camera.updateProjectionMatrix(); this.controls.update();
    if (vector(state.camera?.quaternion, 4)) this.camera.quaternion.fromArray(state.camera.quaternion);
    if (state.layers) this.layers = {...this.layers, ...state.layers};
    for (const values of Object.values(this.objectVisibility)) values.clear();
    for (const [kind, values] of Object.entries(state.objectVisibility || {})) {
      if (!this.objectVisibility[kind] || !Array.isArray(values)) continue;
      for (const item of values) if (Array.isArray(item) && typeof item[0] === 'string') this.objectVisibility[kind].set(item[0], item[1] !== false);
    }
    if (['urdf', 'core'].includes(state.inertiaSource)) this.inertiaSource = state.inertiaSource;
    this.hasFramedModel = state.hasFramedModel === true;
    this.select(state.selection || '');
    this.camera.updateMatrixWorld(); this.requestRender();
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

  geometryJob(data, transfer=[]) {
    if(!this.geometryWorker) {
      const worker=new Worker(new URL('./model-geometry-worker.mjs',import.meta.url),{type:'module'});
      this.geometryWorker=worker;
      worker.onmessage=({data:result})=>{
        const job=this.geometryJobs.get(result.id);if(!job)return;
        this.geometryJobs.delete(result.id);
        result.error?job.reject(new Error(result.error)):job.resolve(result);
      };
      worker.onerror=event=>{
        const error=new Error(event.message||'Model geometry preparation failed');
        for(const job of this.geometryJobs.values())job.reject(error);
        this.geometryJobs.clear();worker.terminate();
        if(this.geometryWorker===worker)this.geometryWorker=null;
      };
    }
    return new Promise((resolve,reject)=>{
      const id=++this.geometryJobSerial;this.geometryJobs.set(id,{resolve,reject});
      try{this.geometryWorker.postMessage({...data,id},transfer);}
      catch(error){this.geometryJobs.delete(id);reject(error);}
    });
  }

  async meshResource(file,generation) {
    if(!this.meshGeometryCache.has(file)) {
      const pending=(async()=>{
        const resource=await this.resourceReader(file);
        if(this.disposed || generation!==this.modelGeneration || resource.extension!=='.stl')return {resource};
        const prepared=await this.geometryJob({kind:'stl',resource:resource.data});
        if(this.disposed || generation!==this.modelGeneration)return {resource};
        const geometry=new THREE.BufferGeometry();
        for(const [name,attribute] of Object.entries(prepared.geometry.attributes))
          geometry.setAttribute(name,new THREE.BufferAttribute(attribute.array,attribute.itemSize,attribute.normalized));
        if(prepared.geometry.index)geometry.setIndex(new THREE.BufferAttribute(prepared.geometry.index.array,1));
        const edges=new THREE.BufferGeometry();edges.setAttribute('position',new THREE.BufferAttribute(prepared.edges,3));
        this.ghostEdges.set(geometry,edges);
        return {resource:{extension:resource.extension},geometry};
      })();
      this.meshGeometryCache.set(file,pending);
    }
    return this.meshGeometryCache.get(file);
  }

  async prepareGhostEdges(object,generation) {
    const geometries=new Set();object.traverse(node=>{if(node.isMesh && node.geometry)geometries.add(node.geometry);});
    for(const geometry of geometries) {
      if(this.ghostEdges.has(geometry))continue;
      // Transfer copies: the measured model retains its full original buffers.
      const position=geometry.getAttribute('position'),index=geometry.index;
      const vertices=new Float32Array(position.count*3);
      for(let i=0;i<position.count;++i){vertices[3*i]=position.getX(i);vertices[3*i+1]=position.getY(i);vertices[3*i+2]=position.getZ(i);}
      const data={attributes:{position:{array:vertices,itemSize:3,normalized:false}},index:index?{array:index.array.slice()}:null};
      const transfer=[data.attributes.position.array.buffer];if(data.index)transfer.push(data.index.array.buffer);
      const prepared=await this.geometryJob({kind:'edges',geometry:data},transfer);
      if(this.disposed || generation!==this.modelGeneration)return;
      const edges=new THREE.BufferGeometry();edges.setAttribute('position',new THREE.BufferAttribute(prepared.edges,3));
      this.ghostEdges.set(geometry,edges);
    }
  }

  ghostLinkGroup(name,original) {
    this.ghostMaterials ||= {
      fill:new THREE.MeshBasicMaterial({color:0x16b8b0,transparent:true,opacity:0.2,depthTest:false,depthWrite:false,side:THREE.DoubleSide}),
      line:new THREE.LineBasicMaterial({color:0x079e98,transparent:true,opacity:0.9,depthTest:false,depthWrite:false})};
    const group=new THREE.Group();group.name=name;group.matrixAutoUpdate=false;
    for(const holder of original.children) {
      if(holder.userData?.layer!=='visual')continue;
      const copy=holder.clone(true),outlines=[];
      copy.traverse(child=>{
        if(!child.isMesh || !child.geometry)return;
        const edges=this.ghostEdges.get(child.geometry);if(!edges)throw new Error('Reference outline geometry is not prepared');
        child.material=this.ghostMaterials.fill;child.renderOrder=24;
        const outline=new THREE.LineSegments(edges,this.ghostMaterials.line);
        outline.renderOrder=25;outline.raycast=()=>{};outlines.push([child,outline]);
      });
      for(const [mesh,outline] of outlines)mesh.add(outline);
      group.add(copy);
    }
    this.ghostLinkGroups.set(name,group);this.ghostRoot.add(group);return group;
  }

  async prepareGhostPose(generation) {
    this.ghostRoot=new THREE.Group();this.ghostRoot.name='reference-start-ghost';this.ghostRoot.visible=false;
    this.scene.add(this.ghostRoot);
    for(const [name,original] of this.linkGroups)this.ghostLinkGroup(name,original);
    // Hidden geometry has no target pose yet. Compile its unchanged overlay
    // materials ahead of the first user preview, without drawing a fake pose.
    await this.renderer.compileAsync(this.ghostRoot,this.camera,this.scene);
    return !this.disposed && generation===this.modelGeneration;
  }

  disposeGeometryPreparation() {
    this.geometryWorker?.terminate();this.geometryWorker=null;
    for(const job of this.geometryJobs.values())job.reject(new Error('Model geometry preparation was cancelled'));
    this.geometryJobs.clear();this.meshGeometryCache.clear();
    for(const edges of this.ghostEdges.values())edges.dispose();this.ghostEdges.clear();
    if(this.ghostMaterials){this.ghostMaterials.fill.dispose();this.ghostMaterials.line.dispose();this.ghostMaterials=null;}
  }

  async setModel(payload) {
    if (this.disposed || !this.running) return false;
    this.disposeRoot();
    const generation = this.modelGeneration;
    this.payload = payload;
    this.modelLoaded = false; this.modelStatus = 'loading'; this.modelErrors = []; this.modelError = null;
    this.emitStatus();
    const frames = new Map((payload.native?.state?.frames || []).map(x => [x.name, x]));
    for (const link of payload.model?.links || []) {
      const group = new THREE.Group(); group.name = link.name; group.userData.kind='link'; group.userData.name=link.name; group.userData.ownerKind='links'; group.userData.ownerName=link.name; this.objectVisibility.links.set(link.name,true);
      const frame = frames.get(link.name); if (frame) group.matrix.copy(frameMatrix(frame));
      group.matrixAutoUpdate = false; this.root.add(group); this.linkGroups.set(link.name, group);
      for (const item of link.visuals || []) {
        await this.addGeometry(group, item, false, generation);
        if (this.disposed || !this.running || generation !== this.modelGeneration) return false;
      }
      for (const item of link.collisions || []) {
        await this.addGeometry(group, item, true, generation);
        if (this.disposed || !this.running || generation !== this.modelGeneration) return false;
      }
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
    this.setLayers(this.layers); this.refreshVisibility();
    this.resetView({immediate: !this.hasFramedModel}); this.hasFramedModel = true;
    if(this.modelErrors.length===0 && !await this.prepareGhostPose(generation))return false;
    if(this.modelErrors.length===0)await this.renderer.compileAsync(this.scene,this.camera);
    if(this.disposed || !this.running || generation!==this.modelGeneration)return false;
    this.modelLoaded = this.modelErrors.length === 0;
    this.modelStatus = this.modelLoaded ? 'ready' : 'failed';
    if (!this.modelLoaded) this.modelError = {category: 'model', code: 'MODEL_RESOURCE_LOAD_FAILED',
      message: this.modelErrors.map(error => `${error.link} / ${error.name || error.layer}: ${error.message}`).join('\n')};
    this.modelStatusPending = true; this.requestRender(); this.emitStatus();
    return this.modelLoaded;
  }

  async addGeometry(group, item, collision, generation = this.modelGeneration) {
    const current = () => !this.disposed && this.running && generation === this.modelGeneration;
    if (!current()) return;
    let object;
    try {
      if (item.error) throw new Error(item.error.message || String(item.error));
      if (!item.geometry) throw new Error('Missing URDF geometry');
      const holder = new THREE.Group(); holder.userData.layer = collision ? 'collision' : 'visual'; holder.matrix.copy(originMatrix(item.origin)); holder.matrixAutoUpdate=false;
      const g = item.geometry;
      if (g.type === 'box') object = new THREE.Mesh(new THREE.BoxGeometry(...g.size), material(item.rgba, collision));
      else if (g.type === 'cylinder') object = new THREE.Mesh(new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32), material(item.rgba, collision));
      else if (g.type === 'sphere') object = new THREE.Mesh(new THREE.SphereGeometry(g.radius, 32, 18), material(item.rgba, collision));
      else if (g.type === 'mesh') {
        const loaded=await this.meshResource(g.path,generation),resource=loaded.resource;
        if (!current()) return;
        if (resource.extension === '.stl') {
          object = new THREE.Mesh(loaded.geometry, material(item.rgba, collision));
        } else if (resource.extension === '.glb') {
          const data = decode(resource.data);
          const gltf = await new Promise((resolve,reject)=>new GLTFLoader().parse(data, '', resolve, reject)); object = gltf.scene;
          if (!current()) { disposeObject(object); return; }
          // The URDF material owns the display appearance; release the imported
          // materials/textures before replacing them instead of retaining them
          disposeObject(object, {geometry: false});
          object.traverse(node => { if (node.isMesh) node.material = material(item.rgba, collision); });
        } else throw new Error(`Unsupported model resource type: ${resource.extension}`);
        object.scale.set(...(g.scale || [1,1,1]));
      } else throw new Error(`Unsupported URDF geometry: ${g.type}`);
      if (!current()) { disposeObject(object); return; }
      let renderable = false;
      object.traverse(node => { if (node.isMesh && node.geometry?.getAttribute('position')?.count >= 3) renderable = true; });
      if (!renderable) throw new Error('Model resource contains no renderable geometry');
      if(!collision)await this.prepareGhostEdges(object,generation);
      if(!current()){disposeObject(object);return;}
      const layer=collision?'collision':'visual'; holder.userData.ownerKind='links'; holder.userData.ownerName=group.name; object.traverse(node=>{if(node.isMesh){node.userData.layer=layer;node.userData.ownerKind='links';node.userData.ownerName=group.name;}}); object.userData.layer=layer; holder.add(object); group.add(holder);
    } catch (error) {
      disposeObject(object);
      if (current()) this.modelErrors.push({link: group.name, name: item.name || '', layer: collision ? 'collision' : 'visual',
        resource: item.geometry?.path || item.geometry?.source || '', message: error?.message || String(error)});
    }
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

  jointTransform(joint, frames) {
    // In URDF the child link frame coincides with its joint frame after motion
    // A static parent * joint origin transform loses revolute and prismatic motion
    const child = frames.get(joint.child);
    if (child) return frameMatrix(child);
    const jointFrame = frames.get(joint.name);
    if (jointFrame) return frameMatrix(jointFrame);
    // Without live frame data, never misplace an inertial body on a moving joint
    if (['revolute', 'continuous', 'prismatic', 'planar', 'floating'].includes(joint.type)) return null;
    const parent = frames.get(joint.parent);
    if (!parent && joint.parent) return null;
    return (parent ? frameMatrix(parent) : new THREE.Matrix4()).multiply(originMatrix(joint.origin));
  }

  addInertia(group, inertial) {
    const xyz=inertial.origin?.xyz||[0,0,0];
    const lineGeometry=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(...xyz)]);
    const line=new THREE.Line(lineGeometry,new THREE.LineBasicMaterial({color:0xff4f87,transparent:true,opacity:0.95,depthTest:false})); line.userData.layer='com'; line.userData.inertiaKind='raw'; group.add(line);
    const local=new THREE.Group(); local.userData.inertiaKind='raw'; local.matrix.copy(originMatrix(inertial.origin)); local.matrixAutoUpdate=false;
    const point=new THREE.Mesh(new THREE.SphereGeometry(0.009,20,14),new THREE.MeshBasicMaterial({color:0xff4f87,depthTest:false})); point.userData.layer='com'; local.add(point);
    const halo=new THREE.Mesh(new THREE.RingGeometry(0.012,0.015,24),new THREE.MeshBasicMaterial({color:0xff4f87,side:THREE.DoubleSide,transparent:true,opacity:0.9,depthTest:false})); halo.userData.layer='com'; local.add(halo);
    const shape=inertiaShape(inertial);
    if(shape){
      const ellipsoid=new THREE.Mesh(new THREE.SphereGeometry(1,24,16),new THREE.MeshBasicMaterial({color:0xff4f87,wireframe:true,transparent:true,opacity:0.52,depthTest:false}));
      ellipsoid.scale.set(...shape.radii); ellipsoid.applyMatrix4(shape.rotation); ellipsoid.userData.layer='inertia'; local.add(ellipsoid);
    }
    group.add(local);
  }

  addEffectiveInertias(payload,frames){
    const joints=new Map((payload.model?.joints||[]).map(j=>[j.name,j]));
    for(const item of payload.native?.info?.effective_inertias||[]){
      const joint=joints.get(item.joint_name); if(!joint)continue; const transform=this.jointTransform(joint,frames); if(!transform)continue;
      const holder=new THREE.Group(); holder.userData.inertiaKind='effective'; holder.userData.ownerKind='links'; holder.userData.ownerName=joint.child; holder.matrix.copy(transform); holder.matrixAutoUpdate=false;
      const local=new THREE.Group(); local.position.set(...(item.center_of_mass||[0,0,0]));
      const point=new THREE.Mesh(new THREE.SphereGeometry(0.013,16,12),new THREE.MeshBasicMaterial({color:0xffb02e,depthTest:false})); point.userData.layer='com'; local.add(point);
      const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(...(item.center_of_mass||[0,0,0]))]),new THREE.LineBasicMaterial({color:0xffb02e,transparent:true,opacity:0.9,depthTest:false})); line.userData.layer='com'; holder.add(line);
      const shape=inertiaShape({valid:true,mass:item.mass,inertia:item.inertia});
      if(shape){const ellipsoid=new THREE.Mesh(new THREE.SphereGeometry(1,20,14),new THREE.MeshBasicMaterial({color:0xffa928,wireframe:true,transparent:true,opacity:0.5,depthTest:false}));ellipsoid.scale.set(...shape.radii);ellipsoid.applyMatrix4(shape.rotation);ellipsoid.userData.layer='inertia';local.add(ellipsoid);}
      holder.add(local); this.root.add(holder); this.effectiveGroups.push({joint,holder});
    }
  }

  updateCenterOfMass(native){
    const com=native?.state?.center_of_mass,previous=this.totalComVisual;
    if(!Array.isArray(com)||com.length!==3){
      if(!previous)return false;
      while(this.comGroup.children.length){const child=this.comGroup.children[0];this.comGroup.remove(child);disposeObject(child);const index=this.labels.indexOf(child);if(index>=0)this.labels.splice(index,1);}
      this.totalComVisual=null;this.requestRender();return true;
    }
    if(previous&&previous.coordinates.every((value,index)=>value===com[index]))return false;
    if(!previous){
      const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3(...com)]),new THREE.LineBasicMaterial({color:0x22d3a6,transparent:true,opacity:0.95,depthTest:false}));line.userData.layer='com';this.comGroup.add(line);
      const point=new THREE.Mesh(new THREE.SphereGeometry(0.014,16,12),new THREE.MeshBasicMaterial({color:0x22d3a6,depthTest:false}));point.position.set(...com);point.userData.layer='com';this.comGroup.add(point);
      const label=this.makeLabel('Total COM',[com[0]+0.025,com[1]+0.025,com[2]+0.025],['com','labels']);this.comGroup.add(label);
      this.totalComVisual={line,point,label,coordinates:com.slice()};this.refreshVisibility();
    }else{
      previous.line.geometry.getAttribute('position').setXYZ(1,...com);previous.line.geometry.getAttribute('position').needsUpdate=true;
      previous.line.geometry.computeBoundingSphere();previous.point.position.set(...com);
      previous.label.position.set(com[0]+0.025,com[1]+0.025,com[2]+0.025);
      previous.coordinates=com.slice();this.requestRender();
    }
    return true;
  }

  // Independent reference pose overlay; never changes the measured robot pose
  clearGhostPose() {
    this.ghostLinkGroups.clear();
    if(!this.ghostRoot)return;
    this.scene.remove(this.ghostRoot);
    // Immutable buffers and both overlay materials belong to the loaded model,
    // so clearing a target never rebuilds outlines or destroys shared resources.
    disposeObject(this.ghostRoot, {geometry:false, textures:false, materials:false});
    this.ghostRoot=null;
    this.requestRender();
  }

  setGhostPose(native) {
    if (this.disposed || !this.running) return;
    const frames=new Map((native?.state?.frames||[]).map(x=>[x.name,x]));
    if(!this.modelLoaded && frames.size)throw new Error('Reference preview model is still loading');
    if(!this.ghostRoot && !frames.size)return;
    if(!this.ghostRoot){
      this.ghostRoot=new THREE.Group();this.ghostRoot.name='reference-start-ghost';
      this.scene.add(this.ghostRoot);
    }
    let visible=0;
    for(const [name,original] of this.linkGroups){
      const frame=frames.get(name);
      let group=this.ghostLinkGroups.get(name);
      if(!group && frame)group=this.ghostLinkGroup(name,original);
      if(!group)continue;
      group.visible=!!frame;
      if(frame){group.matrix.copy(frameMatrix(frame));visible++;}
    }
    this.ghostRoot.visible=visible>0;
    this.refreshVisibility();
  }

  updatePose(native) {
    if (this.disposed || !this.running) return;
    const input=native?.state?.frames||[];
    const unchanged=this.poseFrames?.size===input.length&&input.every(frame=>{
      const previous=this.poseFrames.get(frame.name),position=frame.position||[0,0,0],quaternion=frame.quaternion||[0,0,0,1];
      return previous&&previous.position.every((value,index)=>value===position[index])&&previous.quaternion.every((value,index)=>value===quaternion[index]);
    });
    if(unchanged)return this.updateCenterOfMass(native);
    const frames=new Map(input.map(frame=>[frame.name,frame]));
    this.poseFrames=new Map(input.map(frame=>[frame.name,{position:(frame.position||[0,0,0]).slice(),quaternion:(frame.quaternion||[0,0,0,1]).slice()}]));
    for(const [name,group] of this.linkGroups){const frame=frames.get(name);if(frame)group.matrix.copy(frameMatrix(frame));}
    for(const [name,helper] of this.frameHelpers){const frame=frames.get(name);if(frame)helper.matrix.copy(frameMatrix(frame));}
    for(const entry of this.jointAxes){const transform=this.jointTransform(entry.joint,frames);if(!transform)continue;const position=new THREE.Vector3().setFromMatrixPosition(transform);if(entry.arrow){entry.arrow.position.copy(position);entry.arrow.setDirection(new THREE.Vector3(...(entry.joint.axis||[1,0,0])).normalize().transformDirection(transform));}entry.origin.position.copy(position);entry.label.position.copy(position).add(new THREE.Vector3(0.02,0.02,0.02));}
    for(const entry of this.effectiveGroups){const transform=this.jointTransform(entry.joint,frames);if(transform)entry.holder.matrix.copy(transform);}
    this.updateCenterOfMass(native);this.requestRender();return true;
  }

  clearGravityComparison() {
    for (const group of this.comparisonGroups) {
      group.parent?.remove(group);
      disposeObject(group);
    }
    this.comparisonGroups=[];
    this.requestRender();
  }

  makeComparisonLabel(text, position) {
    const canvas=document.createElement('canvas');
    const ctx=canvas.getContext('2d');ctx.font='600 32px sans-serif';
    const width=Math.min(620,Math.ceil(ctx.measureText(text).width+44));
    canvas.width=width;canvas.height=104;
    ctx.fillStyle='rgba(9,25,40,.92)';ctx.fillRect(0,4,width,94);
    ctx.fillStyle='#38bdf8';ctx.fillRect(0,4,7,94);
    ctx.font='600 32px sans-serif';ctx.fillStyle='#f0f9ff';ctx.fillText(text,22,66);
    const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;
    const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,transparent:true,depthTest:false,depthWrite:false}));
    sprite.position.set(...position);sprite.scale.set(Math.max(width/1700,0.11),0.035,1);
    sprite.renderOrder=75;sprite.userData.layer='candidate';sprite.userData.texture=texture;
    return sprite;
  }

  setGravityComparison(result, options = {}) {
    if (this.disposed || !this.running) return;
    this.clearGravityComparison();
    if (!this.payload || !result?.first_moments?.length) return;
    const scale=options.displayScale??1;
    const markers=gravityComMarkers(this.payload.model?.links,result.first_moments,scale,options.showUnchanged===true,options.sourceInertials??null,options.candidateMasses??null);
    let index=0;
    for(const item of markers){
      const holder=this.linkGroups.get(item.linkName);
      if(!holder)continue;
      const group=new THREE.Group();group.userData.layer='candidate';group.userData.ownerKind='links';group.userData.ownerName=item.linkName;
      const pink=0xff4f87,blue=0x38bdf8;
      const original=new THREE.Vector3(...item.originalCom),candidate=new THREE.Vector3(...item.displayCom);
      const originalPoint=new THREE.Mesh(new THREE.SphereGeometry(0.009,18,14),new THREE.MeshBasicMaterial({color:pink,depthTest:false,depthWrite:false}));
      originalPoint.position.copy(original);originalPoint.renderOrder=66;group.add(originalPoint);
      const originalHalo=new THREE.Mesh(new THREE.SphereGeometry(0.015,16,12),new THREE.MeshBasicMaterial({color:pink,transparent:true,opacity:0.24,depthTest:false,depthWrite:false}));
      originalHalo.position.copy(original);originalHalo.renderOrder=65;group.add(originalHalo);
      const candidatePoint=new THREE.Mesh(new THREE.SphereGeometry(0.009,18,14),new THREE.MeshBasicMaterial({color:blue,depthTest:false,depthWrite:false}));
      candidatePoint.position.copy(candidate);candidatePoint.renderOrder=69;group.add(candidatePoint);
      // Line and visible markers use the same visualization scale; the label always reports the true displacement
      const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([original,candidate]),new THREE.LineBasicMaterial({color:blue,transparent:true,opacity:0.9,depthTest:false,depthWrite:false}));
      line.renderOrder=67;group.add(line);
      const labelOffset=new THREE.Vector3(0.063,(index%2?-.052:.052),0.025);
      const anchor=candidate.clone().add(labelOffset);
      const leaderEnd=anchor.clone().add(new THREE.Vector3(-0.04,0,0));
      const leader=new THREE.Line(new THREE.BufferGeometry().setFromPoints([candidate,leaderEnd]),new THREE.LineBasicMaterial({color:blue,transparent:true,opacity:0.6,depthTest:false,depthWrite:false}));
      leader.renderOrder=68;group.add(leader);
      const label=this.makeComparisonLabel(`${item.linkName}  Δ${item.offsetMm.toFixed(2)} mm`,anchor.toArray());
      group.add(label);
      holder.add(group);this.comparisonGroups.push(group);index++;
    }
    this.refreshVisibility();
  }

  setLayers(layers) {
    this.layers = {...this.layers, ...layers};
    this.refreshVisibility();
  }

  setInertiaSource(source) {
    if (!['urdf', 'core'].includes(source)) return;
    this.inertiaSource = source;
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
    this.requestRender();
  }

  refreshVisibility() {
    if (this.disposed || !this.root) return;
    const updateNode = node => {
      const requiredLayers=Array.isArray(node.userData?.layers) ? node.userData.layers : (node.userData?.layer ? [node.userData.layer] : []);
      let visible=requiredLayers.every(layer => !(layer in this.layers) || !!this.layers[layer]);
      const inertiaKind=node.userData?.inertiaKind;
      if(inertiaKind==='raw') visible=visible && this.inertiaSource==='urdf';
      if(inertiaKind==='effective') visible=visible && this.inertiaSource==='core';
      const kind=node.userData?.ownerKind,name=node.userData?.ownerName;
      if(kind&&name&&this.objectVisibility[kind])visible=visible&&this.objectVisibility[kind].get(name)!==false;
      node.visible=visible;
    };
    this.root.traverse(updateNode);
    // Cached visual holders follow layer/object visibility without changing
    // the frame-presence visibility of their containing ghost Link groups
    this.ghostRoot?.traverse(node => {if(node.userData?.layer)updateNode(node);});
    for(const [name,helper] of this.frameHelpers)helper.visible=!!this.layers.linkFrames&&this.objectVisibility.linkFrames.get(name)!==false;
    this.requestRender();
  }

  select(name) {
    if (this.disposed) return;
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

  installCameraTools() {
    const en = this.language === 'en';
    const buttons = [
      ['fit', en ? 'Fit' : '适应', en ? 'Fit model to view' : '自动适应模型'],
      ['front', en ? 'Front' : '前', en ? 'View along +Y' : '前视　沿 +Y 方向'],
      ['back', en ? 'Back' : '后', en ? 'View along -Y' : '后视　沿 -Y 方向'],
      ['left', en ? 'Left' : '左', en ? 'View along +X' : '左视　沿 +X 方向'],
      ['right', en ? 'Right' : '右', en ? 'View along -X' : '右视　沿 -X 方向'],
      ['top', en ? 'Top' : '顶', en ? 'View along -Z' : '顶视　沿 -Z 方向'],
      ['bottom', en ? 'Bottom' : '底', en ? 'View along +Z' : '底视　沿 +Z 方向'],
    ];
    this.cameraTools = document.createElement('div');
    this.cameraTools.className = 'model-camera-tools';
    this.cameraTools.setAttribute('role', 'group');
    this.cameraTools.setAttribute('aria-label', en ? 'Camera views' : '相机视角');
    for (const [view, label, title] of buttons) {
      const button = document.createElement('button');
      button.type = 'button';
      button.dataset.cameraView = view;
      button.textContent = label;
      button.title = title;
      button.setAttribute('aria-label', title);
      button.addEventListener('click', () => {
        if (view === 'fit') this.resetView();
        else this.setCameraView(view);
      });
      this.cameraTools.appendChild(button);
    }
    this.container.appendChild(this.cameraTools);
    const hint = document.createElement('div');
    hint.className = 'model-camera-hint';
    hint.textContent = en
      ? 'Left: orbit　Right / Shift: pan　Wheel: zoom'
      : '左键旋转　右键 / Shift 平移　滚轮缩放';
    this.cameraHint = hint;
    this.container.appendChild(hint);
  }

  setCameraView(view, distance = null, {target = this.controls.target, immediate = false} = {}) {
    if (this.disposed || !this.running) return;
    const directions = {
      isometric: [1.25, -1.35, 0.95],
      front: [0, -1, 0], back: [0, 1, 0],
      left: [-1, 0, 0], right: [1, 0, 0],
      // A tiny tilt avoids an undefined roll when looking exactly along Z-up
      top: [0, -0.001, 1], bottom: [0, 0.001, -1],
    };
    const direction = directions[view];
    if (!direction) return;
    this.smoothZoom.cancel();
    const current = this.camera.position.distanceTo(target);
    const length = Number.isFinite(distance) && distance > 0 ? distance : Math.max(current, 0.05);
    this.cameraTransition.move(new THREE.Vector3(...direction), length, target, immediate);
    this.requestRender();
  }

  resetView({immediate = false} = {}) {
    if (this.disposed || !this.running) return;
    this.root.updateMatrixWorld(true);
    const box = new THREE.Box3();
    this.root.traverse(node => {
      if (!node.isMesh || node.userData?.layer !== 'visual' || !node.geometry) return;
      node.geometry.computeBoundingBox?.();
      if (node.geometry.boundingBox) box.union(node.geometry.boundingBox.clone().applyMatrix4(node.matrixWorld));
    });
    if (box.isEmpty()) box.setFromObject(this.root);
    if (box.isEmpty()) box.set(new THREE.Vector3(-0.2, -0.2, 0), new THREE.Vector3(0.2, 0.2, 0.4));
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(size.length() * 0.5, 0.12);
    const vfov = THREE.MathUtils.degToRad(this.camera.fov);
    const hfov = 2 * Math.atan(Math.tan(vfov / 2) * Math.max(this.camera.aspect, 0.1));
    const distance = radius / Math.sin(Math.min(vfov, hfov) / 2) * 1.18;
    this.controls.minDistance = Math.max(radius * 0.04, 0.005);
    this.controls.maxDistance = Math.max(distance * 35, 15);
    this.camera.near = Math.max(radius / 2000, 0.0005);
    this.camera.far = Math.max(this.controls.maxDistance * 1.5, 40);
    this.camera.updateProjectionMatrix();
    this.setCameraView('isometric', distance, {target: center, immediate});
  }

  resize() {
    if (this.disposed || !this.running || this.paused || !this.renderer) return;
    this.smoothZoom.cancel();
    const width=this.container.clientWidth, height=this.container.clientHeight;
    if (width <= 0 || height <= 0) { this.suspended = true; this.stopScheduledDraw(); return; }
    try {
      this.renderer.setSize(width,height,false); this.camera.aspect=width/height; this.camera.updateProjectionMatrix();
      this.requestRender();
    } catch (error) { this.failGraphics(error, 'resize'); }
  }

  stopScheduledDraw() {
    if (this.animationFrame != null) cancelAnimationFrame(this.animationFrame);
    if (this.frameTimer != null) clearTimeout(this.frameTimer);
    this.animationFrame = null; this.frameTimer = null;
    this.lastFrameTime = undefined;
  }

  drawable() {
    return !this.paused && !document.hidden && !this.offscreen && this.container.isConnected &&
      this.container.clientWidth > 0 && this.container.clientHeight > 0;
  }

  requestRender() {
    if (this.disposed || !this.running || this.paused) return;
    this.needsRender = true;
    this.suspended = !this.drawable();
    if (this.suspended || this.animationFrame != null || this.frameTimer != null || this.drawing) return;
    this.animationFrame = requestAnimationFrame(next => this.animate(next));
  }

  drawScene() {
    if (this.gl.isContextLost()) throw new Error('WebGL context lost');
    this.renderer.render(this.scene, this.camera);
    if (this.gl.isContextLost()) throw new Error('WebGL context lost during drawing');
    this.drawCount++; this.needsRender = false;
    if (this.status !== 'ready' || this.modelStatusPending) {
      this.status = 'ready'; this.modelStatusPending = false;
      this.diagnostics.pixelRatio = this.renderer.getPixelRatio();
      this.diagnostics.drawingBuffer = [this.canvas.width, this.canvas.height];
      this.emitStatus();
    }
    return true;
  }

  renderOnce() {
    if (this.disposed || !this.running || !this.drawable()) return false;
    try { return this.drawScene(); }
    catch (error) {
      this.failGraphics(error, 'render', this.gl?.isContextLost() ? 'lost' : 'failed');
      return false;
    }
  }

  animate(time = performance.now()) {
    this.animationFrame = null;
    if (this.disposed || !this.running || this.paused) return;
    if (!this.drawable()) { this.suspended = true; this.lastFrameTime = undefined; return; }
    this.suspended = false;
    const interval = this.mode === 'compatibility' ? 1000 / 30 : 0;
    const elapsed = this.lastFrameTime === undefined ? interval : time - this.lastFrameTime;
    if (elapsed + 0.1 < interval) {
      this.frameTimer = setTimeout(() => { this.frameTimer = null; this.requestRender(); }, interval - elapsed);
      return;
    }
    const seconds = this.lastFrameTime === undefined ? 1 / 60 : Math.max(0, Math.min(0.1, elapsed / 1000));
    this.lastFrameTime = time;
    this.drawing = true;
    try {
      let changed = this.controls.update(seconds);
      changed = this.cameraTransition.update(seconds) || changed;
      changed = this.smoothZoom.update(seconds) || changed;
      if (this.needsRender || changed) this.drawScene();
      this.drawing = false;
      // A change event wakes the next frame until OrbitControls' damping is
      // settled; an unchanged scene has no permanently running animation loop
      if (changed || this.cameraTransition.active || this.smoothZoom.active || this.needsRender) this.requestRender();
    } catch (error) {
      this.drawing = false;
      this.failGraphics(error, 'render', this.gl?.isContextLost() ? 'lost' : 'failed');
    }
  }

  disposeRoot() {
    this.modelGeneration++;
    this.poseFrames=null;this.totalComVisual=null;
    this.clearGhostPose();
    this.disposeGeometryPreparation();
    disposeObject(this.root);
    this.scene.remove(this.root); this.root=new THREE.Group(); this.scene.add(this.root); this.linkGroups.clear(); this.frameHelpers.clear(); this.jointAxes=[]; this.effectiveGroups=[]; this.labels=[]; this.comparisonGroups=[]; this.objectVisibility={links:new Map(),linkFrames:new Map(),joints:new Map()}; this.comGroup=new THREE.Group(); this.root.add(this.comGroup);
  }

  releaseResources() {
    this.stopScheduledDraw(); this.modelGeneration++;
    this.poseFrames=null;this.totalComVisual=null;
    this.resizeObserver?.disconnect(); this.intersectionObserver?.disconnect();
    if (this.onVisibilityChange) document.removeEventListener('visibilitychange', this.onVisibilityChange);
    this.smoothZoom?.dispose(); this.cameraTransition?.dispose();
    if (this.controls && this.onControlsChange) for (const event of ['change','start','end','zoom-input']) this.controls.removeEventListener(event, this.onControlsChange);
    this.controls?.dispose(); this.cameraTools?.remove(); this.cameraHint?.remove();
    this.canvas?.removeEventListener('webglcontextcreationerror', this.onCreationError);
    this.canvas?.removeEventListener('webglcontextlost', this.onContextLost);
    this.clearGhostPose(); this.disposeGeometryPreparation(); disposeObject(this.scene); this.scene?.clear();
    this.root?.clear(); this.linkGroups?.clear(); this.frameHelpers?.clear();
    this.jointAxes = []; this.effectiveGroups = []; this.labels = []; this.comparisonGroups = [];
    this.modelLoaded = false; this.modelStatus = 'empty'; this.modelErrors = []; this.modelError = null; this.payload = null;
    this.renderer?.dispose();
    // Detaching the canvas does not release its context immediately. Release
    // it explicitly so repeated retry/navigation cannot exhaust WebGL contexts
    this.gl?.getExtension('WEBGL_lose_context')?.loseContext();
    this.canvas?.remove();
  }

  dispose() {
    if (this.disposed) return;
    this.running = false; this.disposed = true; this.status = 'disposed';
    this.releaseResources();
  }
}
