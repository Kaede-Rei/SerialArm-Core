const fs = require('node:fs');
const path = require('node:path');

const app = path.resolve(__dirname, '..');
const source = path.join(app, 'node_modules', 'three');
const target = path.join(app, 'renderer', 'vendor', 'three');

if (!fs.existsSync(path.join(source, 'build', 'three.module.js'))) {
    throw new Error('three package is missing');
}
fs.rmSync(target, { recursive: true, force: true });
fs.mkdirSync(target, { recursive: true });
fs.cpSync(path.join(source, 'build'), path.join(target, 'build'), { recursive: true });
fs.cpSync(path.join(source, 'examples', 'jsm'), path.join(target, 'jsm'), { recursive: true });

function rewrite(folder) {
    for (const entry of fs.readdirSync(folder, { withFileTypes: true })) {
        const file = path.join(folder, entry.name);
        if (entry.isDirectory()) rewrite(file);
        else if (entry.isFile() && entry.name.endsWith('.js')) {
            const relative = path.relative(path.dirname(file), path.join(target, 'build', 'three.module.js')).replaceAll(path.sep, '/');
            const specifier = relative.startsWith('.') ? relative : './' + relative;
            const original = fs.readFileSync(file, 'utf8');
            const output = original.replaceAll("from 'three'", `from '${specifier}'`).replaceAll('from "three"', `from "${specifier}"`);
            if (output !== original) fs.writeFileSync(file, output);
        }
    }
}
rewrite(path.join(target, 'jsm'));
console.log('THREE_VENDOR_READY');
