'use strict';
// Panel geometry is direct manipulation. The existing motion engine continues
// to own buttons, disclosure feedback and page changes; the renderer stays mounted.
(() => {
    const storageKey = 'serialarm-studio-layout-v1';
    const defaults = {nav:224, tree:224, inspector:288, reference:360, referenceHeight:240};
    let saved = {};
    try { saved = JSON.parse(localStorage.getItem(storageKey) || '{}'); } catch { /* Use the standard layout when local preferences are unavailable. */ }
    const prefs = {widths:{...defaults,...saved?.widths}, focus:saved?.focus === true, workspaces:saved?.workspaces || {}, tree:saved?.tree !== false, inspector:saved?.inspector !== false};
    for (const key of Object.keys(defaults)) if (!Number.isFinite(prefs.widths[key])) prefs.widths[key] = defaults[key];
    let options = {}, drag = null, compactPane = '', workspaceFocus = false, layoutKey = '', resizeFrame = 0;
    const actual = {}, ranges = {}, observed = new Set();
    const $ = selector => document.querySelector(selector);
    const text = (zh,en) => options.language === 'en' ? en : zh;
    const clamp = (value,min,max) => Math.max(min,Math.min(max,value));
    const remember = () => { try { localStorage.setItem(storageKey,JSON.stringify(prefs)); } catch { /* The current session remains adjustable without persistence. */ } };
    const notifyResize = () => options.resize?.();
    function workspacePrefs() {
        const key = options.key || 'workspace';
        return prefs.workspaces[key] ||= {dock:'side',open:true,reference:options.position==='right'?320:360};
    }
    function separator(name,label) {
        const node = document.createElement('div');
        node.className = 'studio-layout-divider'; node.dataset.layoutSplit = name;
        node.setAttribute('role','separator'); node.tabIndex = 0;
        node.setAttribute('aria-label',label); node.setAttribute('aria-orientation','vertical');
        node.title = text('拖动调整；方向键微调，Shift 加速，Home 或双击复位，Esc 取消拖动','Drag to resize; arrow keys adjust, Shift speeds up, Home or double-click resets, Esc cancels dragging');
        return node;
    }
    function action(name,label) {
        const button = document.createElement('button'); button.type = 'button';
        button.className = 'button studio-layout-button'; button.dataset.layoutAction = name;
        button.textContent = label; return button;
    }
    function setRange(name,min,max) {
        ranges[name] = [min,Math.max(min,max)];
        actual[name] = clamp(prefs.widths[name],...ranges[name]);
        document.querySelectorAll(`[data-layout-split="${name}"]`).forEach(node => {
            node.setAttribute('aria-valuemin',String(Math.round(min)));
            node.setAttribute('aria-valuemax',String(Math.round(ranges[name][1])));
            node.setAttribute('aria-valuenow',String(Math.round(actual[name])));
            node.setAttribute('aria-valuetext',`${Math.round(actual[name])} px`);
        });
        return actual[name];
    }
    function ensureShell() {
        const sidebar = $('.studio-shell-sidebar'), body = sidebar?.parentElement;
        if (!sidebar || !body) return;
        if (!body.querySelector(':scope > [data-layout-split="nav"]')) sidebar.after(separator('nav',text('主导航宽度','Main navigation width')));
        const controls = $('.title-actions');
        if (controls && !controls.querySelector('[data-layout-action="focus"]')) {
            const button = action('focus',text('专注','Focus')); button.classList.add('studio-layout-focus'); controls.prepend(button);
        }
        body.dataset.layoutShell = 'true';
        if (!observed.has(body)) { observer.observe(body); observed.add(body); }
    }
    function ensureModel() {
        const grid = $('.model-workspace'), tree = grid?.querySelector('.model-tree'), inspector = grid?.querySelector('.model-inspector');
        if (!grid || !tree || !inspector) return;
        if (!grid.querySelector('[data-layout-split="tree"]')) tree.after(separator('tree',text('模型对象树宽度','Model tree width')));
        if (!grid.querySelector('[data-layout-split="inspector"]')) inspector.before(separator('inspector',text('模型检视器宽度','Model inspector width')));
        const toolbar = $('.model-tools');
        if (toolbar && !toolbar.querySelector('.studio-layout-model-tools')) {
            const tools = document.createElement('div'); tools.className = 'studio-layout-tools studio-layout-model-tools';
            tools.append(action('tree',text('对象树','Objects')),action('inspector',text('检视器','Inspector')));
            const display = toolbar.querySelector('.model-display-options'); if (display) tools.append(display);
            toolbar.append(tools);
        }
        grid.dataset.layoutModel = 'true';
        if (!observed.has(grid)) { observer.observe(grid); observed.add(grid); }
    }
    function ensureWorkspace(workspace,preview,task) {
        const layout = workspace?.querySelector('.studio-workspace-layout');
        if (!layout || !preview || !task) return null;
        if (preview.parentElement !== layout) layout.prepend(preview);
        if (!layout.querySelector(':scope > .studio-layout-divider')) {
            const split = separator('reference',text('模型对照宽度','Model reference width')); preview.after(split);
        }
        const summary = preview.querySelector(':scope > summary');
        if (summary && !summary.querySelector('.studio-layout-tools')) {
            const controls = document.createElement('span'); controls.className = 'studio-layout-tools';
            controls.append(action('dock',text('下方停靠','Dock below')),action('model-focus',text('放大','Expand'))); summary.append(controls);
        }
        if (!preview.dataset.layoutBound) {
            preview.dataset.layoutBound = 'true';
            preview.addEventListener('toggle', () => {
                if (!preview.isConnected || preview.dataset.layoutSync === 'true') return;
                workspacePrefs().open = preview.open; remember(); sync(); options.onPreviewChange?.();
            });
        }
        layout.dataset.layoutReady = 'true';
        if (!observed.has(workspace)) { observer.observe(workspace); observed.add(workspace); }
        return layout;
    }
    function syncShell() {
        const body = $('[data-layout-shell]'), sidebar = body?.querySelector('.studio-shell-sidebar');
        if (!body || !sidebar) return;
        const width = setRange('nav',176,Math.min(360,body.clientWidth - 520));
        sidebar.style.width = `${width}px`;
        body.dataset.layoutFocus = String(prefs.focus);
        const button = $('[data-layout-action="focus"]');
        if (button) { button.textContent = text(prefs.focus?'显示导航':'专注',prefs.focus?'Show navigation':'Focus'); button.setAttribute('aria-pressed',String(prefs.focus)); }
    }
    function syncModel() {
        const grid = $('[data-layout-model]'); if (!grid) return;
        const compact = grid.clientWidth < 840, width = grid.clientWidth;
        const tree = setRange('tree',168,compact ? Math.max(168,width - 72) : Math.min(360,width - (prefs.inspector ? 244 : 0) - 296));
        const inspector = setRange('inspector',244,compact ? Math.max(244,width - 72) : Math.min(480,width - (prefs.tree ? tree : 0) - 296));
        grid.dataset.layoutCompact = String(compact);
        grid.dataset.layoutTree = String(compact ? compactPane === 'tree' : prefs.tree);
        grid.dataset.layoutInspector = String(compact ? compactPane === 'inspector' : prefs.inspector);
        grid.style.setProperty('--layout-tree',`${tree}px`); grid.style.setProperty('--layout-inspector',`${inspector}px`);
        if (!compact) grid.style.gridTemplateColumns = `${prefs.tree?`${tree}px 8px`:'0 0'} minmax(0,1fr) ${prefs.inspector?`8px ${inspector}px`:'0 0'}`;
        else grid.style.gridTemplateColumns = 'minmax(0,1fr)';
        for (const name of ['tree','inspector']) {
            const button = $(`[data-layout-action="${name}"]`);
            button?.setAttribute('aria-pressed',grid.dataset[name === 'tree' ? 'layoutTree' : 'layoutInspector']);
        }
    }
    function syncWorkspace(workspace,preview,task,nextOptions) {
        if (nextOptions) options = {...options,...nextOptions};
        const layout = ensureWorkspace(workspace,preview,task); if (!layout) return;
        const key = options.key || 'workspace';
        if (key !== layoutKey) { workspaceFocus = false; layoutKey = key; }
        const setting = workspacePrefs(), width = layout.clientWidth;
        const sideMinimum = (options.taskMinimum || 590) + 268;
        const dock = width < sideMinimum ? 'bottom' : setting.dock === 'bottom' ? 'bottom' : 'side';
        const compact = dock === 'bottom' && window.innerHeight < 820;
        workspace.dataset.layoutDensity = compact ? 'compact' : 'normal';
        if (preview.dataset.layoutKey !== key) {
            preview.dataset.layoutKey = key;
            preview.dataset.layoutSync = 'true'; preview.open = setting.open !== false;
            queueMicrotask(() => { delete preview.dataset.layoutSync; });
        }
        const headerHeight = preview.querySelector(':scope > summary')?.offsetHeight || 50;
        const taskMinimum = compact ? 200 : 260, modelMinimum = headerHeight + 182;
        layout.style.setProperty('--layout-task-minimum',`${taskMinimum}px`);
        const content = layout.closest('.studio-page-body') || layout.closest('.content');
        const offset = content ? layout.getBoundingClientRect().top - content.getBoundingClientRect().top + content.scrollTop : layout.getBoundingClientRect().top;
        const minimumHeight = !preview.open ? taskMinimum + headerHeight + 2 : workspaceFocus ? modelMinimum : dock === 'bottom' ? taskMinimum + 8 + modelMinimum : Math.max(taskMinimum,modelMinimum);
        const availableHeight = Math.max(minimumHeight,(content?.clientHeight || window.innerHeight) - Math.max(0,offset) - (content?parseFloat(getComputedStyle(content).paddingBottom)||0:12));
        layout.style.setProperty('--layout-workspace-height',`${availableHeight}px`);
        prefs.widths.reference=setting.reference ?? (options.position==='right'?320:360);
        const reference = setRange('reference',260,Math.min(640,width - (options.taskMinimum || 590) - 8));
        prefs.widths.referenceHeight=setting.referenceHeight ?? Math.min(240,availableHeight*.34);
        const referenceHeight = setRange('referenceHeight',modelMinimum,Math.min(480,availableHeight - taskMinimum - 8));
        layout.dataset.layoutPosition=options.position || 'left';
        layout.style.setProperty('--layout-reference',`${reference}px`);
        layout.style.setProperty('--layout-reference-height',`${referenceHeight}px`);
        layout.dataset.layoutDock = dock; layout.dataset.layoutExpanded = String(workspaceFocus && preview.open);
        layout.dataset.layoutOpen = String(preview.open);
        workspace.dataset.narrow = 'false'; workspace.dataset.variant = 'adjustable';
        const split = layout.querySelector(':scope > .studio-layout-divider');
        split.dataset.layoutSplit = dock === 'bottom' ? 'referenceHeight' : 'reference';
        split.setAttribute('aria-orientation',dock === 'bottom' ? 'horizontal' : 'vertical');
        split.setAttribute('aria-label',text(dock === 'bottom'?'模型对照高度':'模型对照宽度',dock === 'bottom'?'Model reference height':'Model reference width'));
        const splitName = split.dataset.layoutSplit;
        split.setAttribute('aria-valuemin',String(ranges[splitName][0])); split.setAttribute('aria-valuemax',String(ranges[splitName][1]));
        split.setAttribute('aria-valuenow',String(Math.round(actual[splitName]))); split.setAttribute('aria-valuetext',`${Math.round(actual[splitName])} px`);
        const dockButton = preview.querySelector('[data-layout-action="dock"]');
        if (dockButton) { dockButton.textContent = text(dock==='bottom'?'侧边停靠':'下方停靠',dock==='bottom'?'Dock beside':'Dock below'); dockButton.disabled = width < sideMinimum; dockButton.title = width < sideMinimum ? text('空间不足时自动停靠下方；放宽工作区后恢复所选布局','Docked below at this width; the chosen layout returns when space allows') : ''; }
        const focusButton = preview.querySelector('[data-layout-action="model-focus"]');
        if (focusButton) { focusButton.textContent = text(workspaceFocus?'返回任务':'放大',workspaceFocus?'Return to task':'Expand'); focusButton.setAttribute('aria-pressed',String(workspaceFocus)); }
        preview.style.setProperty('--layout-preview-header',`${preview.querySelector(':scope > summary')?.offsetHeight || 44}px`);
    }
    function sync() {
        syncShell(); syncModel();
        const workspace = $('.studio-workspace');
        if (workspace) syncWorkspace(workspace,workspace.querySelector('[data-workspace-preview]'),workspace.querySelector('.workbench-task'));
        notifyResize();
    }
    const observer = new ResizeObserver(() => {
        cancelAnimationFrame(resizeFrame); resizeFrame = requestAnimationFrame(sync);
    });
    function mount(nextOptions = {}) {
        for (const node of observed) if (!node.isConnected) { observer.unobserve(node); observed.delete(node); }
        options = {...options,...nextOptions}; ensureShell(); ensureModel(); sync();
    }
    function endDrag(cancel = false) {
        if (!drag) return;
        const current = drag; drag = null;
        if (cancel) {prefs.widths[current.name] = current.previous;if(current.name.startsWith('reference'))workspacePrefs()[current.name]=current.previous;}
        document.documentElement.removeAttribute('data-layout-drag');
        if (current.node.hasPointerCapture(current.id)) current.node.releasePointerCapture(current.id);
        sync(); if (!cancel) remember();
    }
    document.addEventListener('pointerdown',event => {
        const node = event.target.closest?.('[data-layout-split]');
        if (!node || event.button !== 0 || drag) return;
        const name = node.dataset.layoutSplit, axis = name === 'referenceHeight' ? 'y' : 'x';
        event.preventDefault(); node.focus({preventScroll:true}); node.setPointerCapture(event.pointerId);
        drag = {node,name,axis,id:event.pointerId,position:axis === 'x' ? event.clientX : event.clientY,start:actual[name],previous:prefs.widths[name],sign:['inspector','referenceHeight'].includes(name)||(name==='reference'&&options.position==='right')?-1:1};
        document.documentElement.dataset.layoutDrag = axis;
    });
    document.addEventListener('pointermove',event => {
        if (!drag || event.pointerId !== drag.id) return;
        const position = drag.axis === 'x' ? event.clientX : event.clientY;
        prefs.widths[drag.name] = clamp(drag.start + (position-drag.position)*drag.sign,...ranges[drag.name]); if(drag.name.startsWith('reference'))workspacePrefs()[drag.name]=prefs.widths[drag.name]; sync();
    });
    document.addEventListener('pointerup',event => { if (drag?.id === event.pointerId) endDrag(); });
    document.addEventListener('pointercancel',event => { if (drag?.id === event.pointerId) endDrag(true); });
    document.addEventListener('lostpointercapture',event => { if (drag?.id === event.pointerId) endDrag(true); });
    window.addEventListener('blur',() => endDrag(true));
    document.addEventListener('dblclick',event => {
        const node = event.target.closest?.('[data-layout-split]'); if (!node) return;
        prefs.widths[node.dataset.layoutSplit] = defaults[node.dataset.layoutSplit]; if(node.dataset.layoutSplit.startsWith('reference'))workspacePrefs()[node.dataset.layoutSplit]=prefs.widths[node.dataset.layoutSplit]; sync(); remember();
    });
    document.addEventListener('keydown',event => {
        if (event.key === 'Escape' && drag) { event.preventDefault(); endDrag(true); return; }
        const node = event.target.closest?.('[data-layout-split]'); if (!node) return;
        const name = node.dataset.layoutSplit, vertical = name === 'referenceHeight';
        const keys = vertical ? ['ArrowUp','ArrowDown'] : ['ArrowLeft','ArrowRight'];
        if (!keys.includes(event.key) && event.key !== 'Home') return;
        event.preventDefault(); const sign = ['inspector','referenceHeight'].includes(name)||(name==='reference'&&options.position==='right') ? -1 : 1;
        prefs.widths[name] = event.key === 'Home' ? defaults[name] : clamp(actual[name]+(event.key===keys[1]?1:-1)*sign*(event.shiftKey?24:8),...ranges[name]);
        if(name.startsWith('reference'))workspacePrefs()[name]=prefs.widths[name];
        sync(); remember();
    });
    document.addEventListener('click',event => {
        const button = event.target.closest?.('[data-layout-action]'); if (!button) return;
        event.preventDefault(); event.stopPropagation();
        const name = button.dataset.layoutAction;
        if (name === 'focus') prefs.focus = !prefs.focus;
        else if (name === 'tree' || name === 'inspector') {
            if ($('[data-layout-model]')?.dataset.layoutCompact === 'true') compactPane = compactPane === name ? '' : name;
            else prefs[name] = !prefs[name];
        } else if (name === 'dock') workspacePrefs().dock = workspacePrefs().dock === 'bottom' ? 'side' : 'bottom';
        else if (name === 'model-focus') { workspaceFocus = !workspaceFocus; const preview = $('[data-workspace-preview]'); if (preview) preview.open = true; }
        sync(); remember();
    });
    window.addEventListener('resize',() => { cancelAnimationFrame(resizeFrame); resizeFrame = requestAnimationFrame(sync); });
    window.serialArmLayout = {mount,syncWorkspace,sync};
})();
