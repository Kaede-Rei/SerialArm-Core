import * as THREE from './vendor/three/build/three.module.js';
import {STLLoader} from './vendor/three/jsm/loaders/STLLoader.js';

// Parsing and exact 25-degree edge extraction can scan millions of STL
// vertices. Keep that immutable preparation off the interaction thread.
const unpack=data=>{
  const geometry=new THREE.BufferGeometry();
  for(const [name,attribute] of Object.entries(data.attributes))
    geometry.setAttribute(name,new THREE.BufferAttribute(attribute.array,attribute.itemSize,attribute.normalized));
  if(data.index)geometry.setIndex(new THREE.BufferAttribute(data.index.array,1));
  return geometry;
};
const pack=geometry=>({attributes:Object.fromEntries(Object.entries(geometry.attributes).map(([name,attribute])=>
  [name,{array:attribute.array,itemSize:attribute.itemSize,normalized:attribute.normalized}])),
  index:geometry.index?{array:geometry.index.array}:null});
self.onmessage=({data})=>{
  try{
    let geometry;
    if(data.kind==='stl'){
      const raw=atob(data.resource),bytes=new Uint8Array(raw.length);
      for(let i=0;i<raw.length;++i)bytes[i]=raw.charCodeAt(i);
      geometry=new STLLoader().parse(bytes.buffer);
    }else geometry=unpack(data.geometry);
    const edges=new THREE.EdgesGeometry(geometry,25);
    const result={id:data.id,edges:edges.getAttribute('position').array};
    if(data.kind==='stl')result.geometry=pack(geometry);
    const transfers=new Set([result.edges.buffer]);
    if(result.geometry){for(const attribute of Object.values(result.geometry.attributes))transfers.add(attribute.array.buffer);
      if(result.geometry.index)transfers.add(result.geometry.index.array.buffer);}
    self.postMessage(result,[...transfers]);
  }catch(error){self.postMessage({id:data.id,error:error?.message||String(error)});}
};
