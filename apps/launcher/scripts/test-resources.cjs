const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const {synchronizeResources} = require('../desktop/resources.cjs');
test('updates untouched defaults, preserves edits and refreshes executable resources', () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'serialarm-resources-'));
  try {
    const source=path.join(root,'source'), target=path.join(root,'target');fs.mkdirSync(source);
    fs.writeFileSync(path.join(source,'core.yaml'),'old');fs.writeFileSync(path.join(source,'launch.py'),'old');
    assert.deepEqual(synchronizeResources(source,target),[]);
    fs.writeFileSync(path.join(source,'core.yaml'),'new');fs.writeFileSync(path.join(source,'launch.py'),'new');
    assert.deepEqual(synchronizeResources(source,target),[]);
    assert.equal(fs.readFileSync(path.join(target,'core.yaml'),'utf8'),'new');
    fs.writeFileSync(path.join(target,'core.yaml'),'user');fs.writeFileSync(path.join(source,'core.yaml'),'next');
    assert.equal(synchronizeResources(source,target).length,1);
    assert.equal(fs.readFileSync(path.join(target,'core.yaml'),'utf8'),'user');
    assert.equal(fs.readFileSync(path.join(target,'core.yaml.incoming'),'utf8'),'next');
    assert.equal(synchronizeResources(source,target).length,1);
  } finally {fs.rmSync(root,{recursive:true,force:true});}
});
