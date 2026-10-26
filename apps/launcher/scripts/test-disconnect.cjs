const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname, '../renderer/app.js'), 'utf8');
function context(session, telemetry) {
  const state = {session, telemetry, language:'zh-CN'};
  const sandbox = {state, workbenchConnected:() => ['running','stopping'].includes(state.session?.state), t:key => key};
  vm.createContext(sandbox);
  sandbox.workbenchRecoveryRequired = () => sandbox.workbenchConnected() && state.session?.recovery_required === true;
  for (const name of ['workbenchForceDisconnectAvailable','workspaceSessionActionHtml']) {
    const match = source.match(new RegExp('function '+name+'\\([^]*?\\n\\}'));
    assert.ok(match, name);
    vm.runInContext(match[0], sandbox);
  }
  return sandbox;
}
test('faulted responsive session offers normal retry and explicit force disconnect', () => {
  const ui=context({state:'running'}, {robot_state:'FAULT'});
  assert.equal(ui.workbenchForceDisconnectAvailable(), true);
  assert.match(ui.workspaceSessionActionHtml(), /id="workbench-force-disconnect"/);
  assert.match(ui.workspaceSessionActionHtml(), /id="stop-workbench"/);
});
test('shutdown rejection exposes exit without pretending hardware is disabled', () => {
  const ui=context({state:'running',disconnect_blocked:true}, {robot_state:'ACTIVE'});
  assert.equal(ui.workbenchForceDisconnectAvailable(), true);
  assert.equal(ui.state.telemetry.robot_state, 'ACTIVE');
});
test('healthy and disconnected sessions do not show force exit', () => {
  assert.equal(context({state:'running'}, {robot_state:'INACTIVE'}).workbenchForceDisconnectAvailable(),false);
  assert.equal(context({state:'exited'}, {robot_state:'FAULT'}).workbenchForceDisconnectAvailable(),false);
});

async function closeAfterFailedDisable(forceResponse) {
  const main=fs.readFileSync(require('node:path').join(__dirname,'../desktop/main.cjs'),'utf8');
  const begin=main.indexOf('async function closeWindow()');
  const end=main.indexOf('\nfunction packagedBackend()',begin);
  const calls=[];let choices=0,destroyed=false;
  const sandbox={closing:false,bridge:{session:{state:'running',mode:'workbench'},
    request:async(method,params)=>{calls.push({method,params});if(!params?.force)throw new Error('DISABLE_FAILED');sandbox.bridge.session.state='exited';},
    close:async()=>{}},
    win:{getBounds:()=>({width:1200,height:800}),destroy:()=>{destroyed=true;}},
    dialog:{showMessageBox:async()=>({response:choices++===0?0:forceResponse})},
    waitForSessionExit:async()=>{},savePrefs:()=>{},app:{quit:()=>{}}};
  vm.createContext(sandbox);
  vm.runInContext(main.slice(begin,end)+';globalThis.runClose=closeWindow;',sandbox);
  await sandbox.runClose();
  return {calls,destroyed,closing:sandbox.closing};
}
test('window close offers explicit force exit after rejected disable',async()=>{
  const result=await closeAfterFailedDisable(0);
  assert.equal(result.calls.length,2);
  assert.equal(result.calls[1].method,'workbench_stop');
  assert.equal(result.calls[1].params.force,true);
  assert.equal(result.destroyed,true);
  assert.equal(result.closing,false);
});
test('canceling force exit preserves the active session and window',async()=>{
  const result=await closeAfterFailedDisable(1);
  assert.equal(result.calls.length,1);
  assert.equal(result.destroyed,false);
  assert.equal(result.closing,false);
});
