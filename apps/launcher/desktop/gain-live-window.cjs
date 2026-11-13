const path = require('node:path');
const {pathToFileURL} = require('node:url');

// One read-only child of the existing desktop session; it cannot call the bridge.
class GainLiveWindow {
    constructor({BrowserWindow, ipcMain, owner, trusted, directory}) {
        this.window = null; this.snapshot = null;
        this.entry = path.resolve(directory, '../renderer/gain-live.html');
        this.open = async snapshot => {
            this.update(snapshot);
            if (this.window && !this.window.isDestroyed()) {
                if (this.window.isMinimized()) this.window.restore();
                this.window.focus(); return;
            }
            const child = this.window = new BrowserWindow({parent:owner,width:880,height:620,minWidth:480,minHeight:360,
                show:false,title:'实时执行增益',webPreferences:{preload:path.join(directory,'gain-live-preload.cjs'),
                    contextIsolation:true,nodeIntegration:false,sandbox:true}});
            child.setMenuBarVisibility(false);
            child.webContents.setWindowOpenHandler(() => ({action:'deny'}));
            child.webContents.on('will-navigate', event => event.preventDefault());
            child.on('closed', () => { if(this.window===child)this.window=null; });
            child.once('ready-to-show', () => {if(!child.isDestroyed())child.show();});
            await child.loadFile(this.entry);
        };
        ipcMain.handle('launcher:gain-live-open', (event,snapshot) => {trusted(event);return this.open(snapshot);});
        ipcMain.on('launcher:gain-live-update', (event,snapshot) => {trusted(event);this.update(snapshot);});
        ipcMain.handle('launcher:gain-live-current', event => {
            if(!this.window || event.sender!==this.window.webContents || event.senderFrame.url!==pathToFileURL(this.entry).href)
                throw new Error('Untrusted gain monitor origin');
            return this.snapshot;
        });
        owner.on('closed', () => {if(this.window&&!this.window.isDestroyed())this.window.destroy();this.window=null;});
    }
    update(snapshot) {
        if(snapshot?.connected===false && !snapshot.rows?.length && this.snapshot?.rows?.length) {
            snapshot={...this.snapshot,connected:false,valid:false,language:snapshot.language,theme:snapshot.theme};
        }
        this.snapshot=snapshot;
        if(this.window&&!this.window.isDestroyed())this.window.webContents.send('launcher:gain-live-snapshot',snapshot);
    }
}
module.exports={GainLiveWindow};
