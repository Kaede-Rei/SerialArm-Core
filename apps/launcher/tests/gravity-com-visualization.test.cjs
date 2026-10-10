const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const root=path.resolve(__dirname,'../../..');
const read=rel=>fs.readFileSync(path.join(root,rel),'utf8');
const calculate=async()=> (await import('../renderer/gravity-com-geometry.mjs')).gravityComMarkers;

const links=[
  {name:'Link2',inertial:{valid:true,mass:1.07,origin:{xyz:[-0.13049,0.00041,-0.0276]}}},
  {name:'Link3',inertial:{valid:true,mass:0.6104,origin:{xyz:[0.18543,0.05681,-0.03431]}}},
  {name:'Link1',inertial:{valid:true,mass:0.0898,origin:{xyz:[0,0,0.02]}}},
  {name:'tool0',inertial:{valid:false,origin:{xyz:[0,0,0]}}},
];
const moments=[
  {link_name:'Link2',value:[-0.12868, -0.00163, -0.0276].map(x=>x*1.07)},
  {link_name:'Link3',value:[0.18776, 0.05566, -0.03431].map(x=>x*0.6104)},
  {link_name:'Link1',value:[0,0,0.02].map(x=>x*0.0898)},
  {link_name:'tool0',value:[1,2,3]},
];

test('candidate COM renders in link-local coordinates without changing physical values',async()=>{
  const build=await calculate();
  const actual=build(links,moments,1);
  const amplified=build(links,moments,12);
  assert.deepEqual(actual.map(x=>x.linkName),['Link2','Link3']);
  assert.equal(actual.length,amplified.length);
  for(let i=0;i<actual.length;i++){
    const a=actual[i],b=amplified[i];
    assert.deepEqual(a.candidateCom,b.candidateCom);
    assert.deepEqual(a.originalCom,b.originalCom);
    assert.equal(a.offsetMm,b.offsetMm);
    a.originalCom.forEach((v,axis)=>{
      assert.ok(Math.abs(a.displayCom[axis]-a.candidateCom[axis])<1e-12);
      assert.ok(Math.abs(b.displayCom[axis]-(v+12*(b.candidateCom[axis]-v)))<1e-12);
    });
  }
  assert.ok(actual[0].offsetMm>2 && actual[0].offsetMm<3);
});

test('unchanged links are optional and inertial-less frames are always excluded',async()=>{
  const build=await calculate();
  assert.deepEqual(build(links,moments,25,true).map(x=>x.linkName),['Link2','Link3','Link1']);
  assert.deepEqual(build(links,moments,1,false).map(x=>x.linkName),['Link2','Link3']);
  assert.throws(()=>build(links,moments,1e12),/invalid COM visualization scale/);
  assert.deepEqual(build(links,[{link_name:'Link2',value:[NaN,1,2]}],12),[]);
});

test('calibration workspace enables legend controls and redraws after remount',()=>{
  const app=read('apps/launcher/renderer/app.js');
  const model=read('apps/launcher/renderer/model-view.js');
  const css=read('apps/launcher/renderer/styles.css');
  assert.match(app,/calibrationComOverlayHtml\(\)/);
  assert.match(app,/data-calibration-com-scale/);
  assert.match(app,/modelCalibrationComparisonView === modelView/);
  assert.match(app,/modelCalibrationModelBinding\(\)/);
  assert.match(model,/makeComparisonLabel\(`\$\{item.linkName\}/);
  assert.match(model,/new THREE\.Line\(/);
  assert.match(css,/\.com-swatch\.original\{background:#ff4f87/);
  assert.match(css,/\.com-swatch\.candidate\{background:#38bdf8/);
});

test('historical candidate COM uses recorded mass, while the pink COM uses current inertial',async()=>{
  const build=await calculate();
  const current=[{name:'Link6',inertial:{valid:false,mass:0.774251897736,origin:{xyz:[0.000949845843049,-0.00724726249031,0.0280370075459]}}}];
  const history={Link6:{mass:0.785649631359557,com:[-0.000485908838923044,0.00835307348685735,0.0378063045460189]}};
  const fitted=[{link_name:'Link6',value:[0.00340,0.00791,0.03531].map(v=>v*history.Link6.mass)}];
  const markers=build(current,fitted,12,false,history);
  assert.equal(markers.length,1);
  const a=markers[0];
  assert.equal(a.linkName,'Link6');
  assert.deepEqual(a.originalCom,current[0].inertial.origin.xyz);
  for(let i=0;i<3;i++) assert.ok(Math.abs(a.candidateCom[i]-[0.00340,0.00791,0.03531][i])<1e-12);
  assert.ok(a.offsetMm>1);
  assert.equal(build(current,fitted,12,false,{}).length,0);
});

test('read-only fingerprint reconciliation is registered end to end',()=>{
  const main=read('apps/launcher/desktop/main.cjs');
  const bridge=read('apps/launcher/backend/bridge.py');
  const app=read('apps/launcher/renderer/app.js');
  assert.match(main,/'model_calibration_model_alignment'/);
  assert.match(bridge,/compare_calibration_urdfs/);
  assert.match(app,/alignHistoricalCalibrationPreview/);
  assert.match(app,/sourceInertials: binding.historical/);
  assert.match(app,/历史模型对照 · 仅预览/);
});
