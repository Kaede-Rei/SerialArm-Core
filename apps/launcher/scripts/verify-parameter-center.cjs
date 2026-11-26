/* Real renderer + real scoped Profile persistence, restricted to temporary files. */
const fs=require('node:fs'),path=require('node:path'),os=require('node:os');
const {spawnSync}=require('node:child_process');
const {app,BrowserWindow}=require('electron');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const out=fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-parameter-center-'));
const fixture=buildFixture(path.join(out,'fixture'));
const dataPath=path.join(fixture.directory,'data.json'),data=JSON.parse(fs.readFileSync(dataPath));
const program=path.join(__dirname,'parameter-center-fixture.py'),home=path.join(out,'profile');
const result=spawnSync('python3',[program,home,'initialize'],{input:JSON.stringify({editor:data.editor}),encoding:'utf8'});
if(result.status!==0)throw Error(result.stderr);
data.editor=JSON.parse(result.stdout);data.info.available.model=false;
fs.writeFileSync(dataPath,JSON.stringify(data));
let preload=fs.readFileSync(fixture.preload,'utf8');
preload=preload.replace("if(name==='status')",`if(['profile_editor_save','profile_editor_load'].includes(name)){const r=require('node:child_process').spawnSync('python3',[${JSON.stringify(program)},${JSON.stringify(home)},name],{input:JSON.stringify(params||{}),encoding:'utf8'});if(r.status!==0)throw Error(r.stderr);return JSON.parse(r.stdout);}if(name==='inspect')return data.info;if(name==='readiness')return {state:'configuring',checks:{},evidence:{}};if(name==='status')`);
fs.writeFileSync(fixture.preload,preload);
const renderScript=fs.readFileSync(path.join(__dirname,'render-studio-ui.cjs'),'utf8');
const scene=renderScript.slice(renderScript.indexOf('function setScene('),renderScript.indexOf('function snapshot('));
app.setPath('userData',path.join(out,'userdata'));
app.whenReady().then(async()=>{
 const win=new BrowserWindow({width:1440,height:900,frame:false,show:false,webPreferences:{preload:fixture.preload,sandbox:false,offscreen:true,backgroundThrottling:false}});
 try{
  await win.loadFile(fixture.index);await win.webContents.executeJavaScript('document.fonts.ready');
  await win.webContents.executeJavaScript(scene+`;setScene('parameters-hardware','light');bindProfileCenter();`);
  const value=await win.webContents.executeJavaScript(`(async()=>{
   const assert=(value,message)=>{if(!value)throw Error(message);};
   assert(document.querySelectorAll('[data-profile-group]').length===6,'Six central groups');
   for(const theme of ['light','dark']) {
    state.theme=theme;applyTheme();const header=document.querySelector('.profile-center .setup-save-context'),card=header.closest('.studio-workspace-task');
    assert(getComputedStyle(header).backgroundColor===getComputedStyle(card).backgroundColor,'Save toolbar matches its card surface in '+theme);
    assert(getComputedStyle(header,'::before').backgroundColor===getComputedStyle(card).backgroundColor,'Sticky toolbar cover matches its card in '+theme);
   }
   state.theme='light';applyTheme();
   const original=structuredClone(profileSavedValues()),bus=Object.keys(state.profileEditor.hardware.buses)[0];
   state.profileEditor.hardware.buses[bus].device='/unfinished-draft';state.profileDirty=true;
   const joint=state.profileEditor.joint_names[0],path=['core','control','controller','rigid','kp',joint];
   profileAssignPath(state.profileEditor,path,profileGet(state.profileEditor,path)+1);
   state.profileFieldErrors={'core.safety_policy.cmd_vel_scale':'unfinished numeric input'};state.profileRawInputs={'core.safety_policy.cmd_vel_scale':'-'};
   render();bindProfileCenter();openProfileReview('motion');bindProfileCenter();
   assert(document.querySelectorAll('[data-profile-review-field]').length===3,'Changes from separate groups share one review');
   assert(state.profileReviewSelection.length===1,'Current-group save selects only its fields');
   await saveProfileSelected([path]);
   assert(state.profileEditor.hardware.buses[bus].device==='/unfinished-draft','Other draft survives scoped save');
   assert(profileSavedValues().hardware.buses[bus].device===original.hardware.buses[bus].device,'Unselected hardware was not saved');
   assert(profileGet(profileSavedValues(),path)===profileGet(original,path)+1,'Selected gain really saved');
   assert(state.profileRawInputs['core.safety_policy.cmd_vel_scale']==='-','Unfinished invalid input survives scoped save');
   assert(profileChanges().length===2,'Only remaining unsaved changes remain');
   state.profileCenterView='edit';state.profileSearchQuery='';state.profileGroup='hardware';render();bindProfileCenter();
   const search=document.querySelector('#profile-search');search.value='joint2 direction';search.dispatchEvent(new Event('input',{bubbles:true}));
   assert(document.querySelectorAll('[data-profile-search-target]').length===1,'Joint and field search resolves exact field');
   assert(document.querySelector('[data-profile-search-target]').dataset.profileSearchTarget==='core.calibration.joints.joint2.direction','Search retains stable field identity');
   openProfileReview();bindProfileCenter();document.querySelector('[data-profile-revert]').click();bindProfileCenter();
   assert(profileChanges().length===1,'Field revert preserves other edits');
   assert(profileHasUnsavedChanges(),'Invalid raw input counts as an unsaved draft');
   const fresh={...structuredClone(state.profileEditor),...structuredClone(profileSavedValues())};
   fresh.core.model.gravity_scale[joint]=1.15;fresh.saved_values=structuredClone({core:fresh.core,hardware:fresh.hardware,profile_entry:fresh.profile_entry});
   rebaseProfileDraft(fresh);
   assert(state.profileEditor.core.model.gravity_scale[joint]===1.15,'Fresh saved values are not overwritten by stale untouched draft fields');
   assert(profileChanges().length===1,'Rebase retains only actual unsaved edits');
   state.profileEditor.core.control.hold_compensation={enabled:false,integral_gain:{[joint]:.15}};
   delete state.profileEditor.saved_values.core.control.hold_compensation;
   const added=profileChanges().filter(item=>item.id.startsWith('core.control.hold_compensation.'));
   assert(added.length===2&&added.every(item=>item.group==='motion'),'Added parameter blocks produce exact field changes in the right group');
   return {groups:6,scopedSave:true,remainingDraft:true,invalidDraftRetained:true,search:true,revert:true};
  })()`);
  console.log(JSON.stringify({out,...value}));win.destroy();app.quit();
 }catch(error){console.error(error);win.destroy();app.exit(1);}
});
