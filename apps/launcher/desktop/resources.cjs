const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const digest = file => crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex');
function synchronizeResources(source, destination) {
  fs.mkdirSync(destination, {recursive: true});
  const receipt = path.join(destination, '.defaults.json');
  let previous = {};
  try { previous = JSON.parse(fs.readFileSync(receipt, 'utf8')); } catch {}
  const next = {}, conflicts = [];
  function walk(directory) {
    for (const item of fs.readdirSync(directory, {withFileTypes: true})) {
      const from = path.join(directory, item.name);
      if (item.isDirectory()) { walk(from); continue; }
      const relative = path.relative(source, from);
      const to = path.join(destination, relative);
      const incoming = digest(from);
      next[relative] = incoming;
      const exists = fs.existsSync(to);
      const editable = /\.(yaml|yml|urdf|xacro|srdf)$/i.test(relative);
      if (exists && editable && digest(to) !== incoming && digest(to) !== previous[relative]) {
        const candidate = to + '.incoming';
        fs.mkdirSync(path.dirname(candidate), {recursive: true});
        fs.copyFileSync(from, candidate);
        conflicts.push({path: to, candidate});
        // Keep the original baseline until the user resolves this conflict
        next[relative] = previous[relative] || null;
        continue;
      }
      fs.mkdirSync(path.dirname(to), {recursive: true});
      if (!exists || digest(to) !== incoming) fs.copyFileSync(from, to);
      if (fs.existsSync(to + '.incoming')) fs.unlinkSync(to + '.incoming');
    }
  }
  walk(source);
  for (const relative of Object.keys(previous)) {
    if (relative in next) continue;
    const target = path.join(destination, relative);
    if (fs.existsSync(target) && digest(target) === previous[relative]) fs.unlinkSync(target);
  }
  fs.writeFileSync(receipt + '.tmp', JSON.stringify(next, null, 2));
  fs.renameSync(receipt + '.tmp', receipt);
  return conflicts;
}
module.exports = {synchronizeResources};
