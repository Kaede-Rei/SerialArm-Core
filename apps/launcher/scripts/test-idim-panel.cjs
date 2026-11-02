const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const source=fs.readFileSync(require('node:path').join(__dirname,'../renderer/app.js'),'utf8');
function panel(report){
 const sandbox={state:{language:'zh-CN',modelCalibrationDirectory:'/task',idimReport:report,idimEstimator:'WLS',idimScope:'global',idimCutoff:5},currentModelCalibration:()=>({task_id:'test'}),workbenchConnected:()=>false,calibrationSelectableLinks:()=>[{name:'Link1'},{name:'Link2'}],esc:x=>String(x??'')};vm.createContext(sandbox);
 for(const name of ['idimResultHtml','idimAuditHtml','idimExportHtml','idimPanelHtml']){const match=source.match(new RegExp('function '+name+'\\([^]*?\\n\\}'));assert.ok(match);vm.runInContext(match[0],sandbox);}
 return sandbox.idimPanelHtml();
}
test('motion improvement alone never enables validated model export',()=>{
 const html=panel({task_id:'test',exportable:false,physical_pass:false,validation_motion:{baseline:{rms_nm:.2},candidate:{rms_nm:.14}},gravity_validation:{baseline_rms_nm:.04,candidate_rms_nm:.5},blocking_reasons:['重力未通过']});
 assert.match(html,/id="idim-export"[^>]*disabled/);assert.match(html,/0\.20000 → 0\.14000/);assert.match(html,/0\.04000 → 0\.50000/);assert.match(html,/重力未通过/);
});
test('another task cannot contribute parameters or export permissions',()=>{
 const html=panel({task_id:'other',exportable:true,physical_pass:true});assert.match(html,/id="idim-export"[^>]*disabled/);assert.doesNotMatch(html,/完整参数对比/);
});
test('missing gravity scores display unknown rather than zero',()=>{
 const html=panel({task_id:'test',exportable:false,gravity_validation:{baseline_rms_nm:null,candidate_rms_nm:null}});assert.match(html,/纯重力项 RMS[^]*?— → —/);
});
test('physical review model and report export remain separate from validation',()=>{
 const html=panel({task_id:'test',review_exportable:true,exportable:false,physical_pass:true,recommendation:'keep_source_prior',candidate_changed:true});
 assert.match(html,/推荐保留原模型/);assert.match(html,/id="idim-review-export" class="button" >/);assert.match(html,/id="idim-export"[^>]*disabled/);assert.match(html,/id="idim-report-export" class="button" >/);
});
