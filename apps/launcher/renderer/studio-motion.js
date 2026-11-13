'use strict';
// Retarget the current presentation value and velocity; no action waits for motion.
(() => {
    const reduced = matchMedia('(prefers-reduced-motion: reduce)');
    const motionReduced = () => reduced.matches || document.documentElement.dataset.reduceMotion === 'true';
    // ATIL quick (900/.7) and panel (560/1) pacing, normalized to damping
    // ratio 1: no bounce, and current value/velocity survive a reversal.
    const quickResponse = 2 * Math.PI / Math.sqrt(900 / .7);
    const panelResponse = 2 * Math.PI / Math.sqrt(560);
    const motions = new Map(), anchors = new WeakMap();
    let pressed = null, frame = 0, previous = 0;
    function tick(now) {
        if (motionReduced()) { resetMotion(); return; }
        const dt = Math.max(0,(now - previous) / 1000);
        previous = now;
        for (const [key, motion] of motions) {
            if (!motion.node.isConnected) { motions.delete(key); continue; }
            if (motion.measureTarget) { motion.target=motion.measureTarget(); motion.staticValue=motion.target; motion.max=Math.max(motion.value,motion.target); }
            // Exact critically damped step avoids frame-rate dependent overshoot.
            const omega = 2 * Math.PI / motion.response;
            const offset = motion.value - motion.target, carry = motion.velocity + omega * offset;
            const decay = Math.exp(-omega * dt);
            motion.value = motion.target + (offset + carry * dt) * decay;
            motion.velocity = (motion.velocity - omega * carry * dt) * decay;
            const bounded = Math.max(motion.min, Math.min(motion.max, motion.value));
            if (bounded !== motion.value) { motion.value = bounded; motion.velocity = 0; }
            motion.apply(motion.value);
            if (Math.abs(motion.value - motion.target) < (motion.positionEpsilon ?? .0001) && Math.abs(motion.velocity) < (motion.velocityEpsilon ?? .001)) {
                motion.apply(motion.target); motion.settled?.(motion.target); motions.delete(key);
            }
        }
        frame = motions.size ? requestAnimationFrame(tick) : 0;
    }
    function move(node, target, options) {
        const key = options.channel || node;
        if (motionReduced()) { motions.delete(key); options.apply(options.staticValue ?? target); options.settled?.(options.staticValue ?? target); return; }
        const motion = motions.get(key) || {value:options.from,velocity:0};
        Object.assign(motion, options, {target,node}); motions.set(key, motion);
        if (Math.abs(motion.value-target)<(motion.positionEpsilon ?? .0001) && Math.abs(motion.velocity)<(motion.velocityEpsilon ?? .001)) {
            motion.apply(target); motion.settled?.(target); motions.delete(key); return;
        }
        motion.apply(motion.value);
        if (!frame) { previous = performance.now(); frame = requestAnimationFrame(tick); }
    }
    function respond(button, target) {
        move(button, target, {from:Number(getComputedStyle(button).scale) || 1,response:quickResponse,min:.97,max:1,staticValue:1,
            apply:value => { button.style.scale = String(value); },
            settled:value => { if (value === 1) button.style.removeProperty('scale'); }});
    }
    function release() { if (pressed?.isConnected) respond(pressed, 1); pressed = null; }
    function press(event) {
        if (motionReduced() || event instanceof PointerEvent && event.button !== 0) return;
        if (event instanceof KeyboardEvent && (event.isComposing || event.repeat || ![' ', 'Enter'].includes(event.key))) return;
        const button = event.target instanceof Element ? event.target.closest('button') : null;
        if (!button || button.disabled || button.closest('[inert]') || button.dataset.motion === 'none') return;
        if (!button.matches('.primary,[data-motion="press"]') ||
            button.hasAttribute('aria-pressed') || button.hasAttribute('aria-current') ||
            button.matches('.danger,.title-button') ||
            button.closest('.robot-operation-bar,.task-window-actions,#operation-safety-actions')) return;
        if (event instanceof KeyboardEvent && event.target !== button) return;
        release(); pressed = button; respond(button, button.clientWidth > 160 ? .993 : .985);
    }
    document.addEventListener('pointerdown', press, true);
    document.addEventListener('pointerup', release, true);
    document.addEventListener('pointercancel', release, true);
    document.addEventListener('keydown', press, true);
    document.addEventListener('keyup', event => { if (!event.isComposing && [' ', 'Enter'].includes(event.key)) release(); }, true);
    window.addEventListener('blur', release);
    function resetMotion() {
        if (!motionReduced()) return;
        pressed = null;
        for (const motion of motions.values()) { const value=motion.staticValue ?? motion.target; motion.apply(value); motion.settled?.(value); }
        motions.clear(); cancelAnimationFrame(frame); frame = 0;
    }
    reduced.addEventListener('change', resetMotion);
    new MutationObserver(resetMotion).observe(document.documentElement, {attributes:true,attributeFilter:['data-reduce-motion']});
    function enterPage(page) {
        const current = motions.get(page);
        move(page, 1, {from:current?.value ?? .94,response:panelResponse,min:0,max:1,staticValue:1,
            apply:value => { page.style.opacity = String(value); },
            settled:() => page.style.removeProperty('opacity')});
    }
    function anchorPopover(menu, trigger) {
        const bound = anchors.get(menu);
        if (bound) { bound.trigger = trigger; return; }
        const anchor = {trigger}; anchors.set(menu, anchor);
        menu.dataset.motionAnchored = 'true';
        function animate(open) {
            const source = anchor.trigger.getBoundingClientRect();
            const left = parseFloat(menu.style.left) || menu.getBoundingClientRect().left;
            const top = parseFloat(menu.style.top) || menu.getBoundingClientRect().top;
            const above = top + menu.offsetHeight <= source.top + 1;
            const origin = Math.max(0,Math.min(menu.offsetWidth,source.left + source.width / 2 - left));
            menu.style.transformOrigin = `${origin}px ${above ? '100%' : '0%'}`;
            menu.dataset.motionPlacement = above ? 'above' : 'below';
            move(menu, open ? 1 : 0, {from:motions.get(menu)?.value ?? (open ? 0 : 1),response:quickResponse,min:0,max:1,
                apply:value => { menu.style.opacity=String(value); menu.style.transform=motionReduced()?'none':`translateY(${(above ? 4 : -4)*(1-value)}px) scale(${.97+.03*value})`; }});
        }
        menu.addEventListener('beforetoggle', event => {
            const open = event.newState === 'open'; menu.inert = !open;
            if (open) { menu.style.removeProperty('pointer-events'); menu.style.opacity=String(motions.get(menu)?.value ?? 0); }
            else { menu.style.pointerEvents='none'; animate(false); if (menu.contains(document.activeElement)) anchor.trigger.focus({preventScroll:true}); }
        });
        menu.addEventListener('toggle', event => { if (event.newState === 'open') animate(true); });
    }
    // Keep native details and their editors mounted. The open attribute changes
    // immediately; only the visual height is animated, never the user's action.
    const disclosures = new WeakMap();
    const detailsSupported = CSS.supports('selector(details::details-content)');
    function rememberDisclosure(node) {
        let record = disclosures.get(node);
        if (!record) {
            const height=node.getBoundingClientRect().height;
            record = {open:node.open,height,openHeight:node.open?height:0,closedHeight:node.open?null:height,progress:node.open?1:0,progressChannel:{},summaryGap:parseFloat(getComputedStyle(node.firstElementChild).marginBottom) || 0,inert:new Map()};
            disclosures.set(node, record); node.dataset.studioDisclosure = 'true';
        }
        return record;
    }
    function restoreDisclosureChildren(record) {
        for (const [child, inert] of record.inert) child.inert = inert;
        record.inert.clear();
    }
    function animateDisclosure(node, open, currentHeight = null) {
        const record = rememberDisclosure(node), summary = node.firstElementChild;
        const from = motions.get(node)?.value ?? currentHeight ?? record.height;
        const gap = record.open === node.open ? parseFloat(getComputedStyle(summary).marginBottom) || 0 : record.summaryGap;
        if (!open) {
            if (node.contains(document.activeElement) && !summary.contains(document.activeElement)) summary.focus({preventScroll:true});
            for (const child of node.children) if (child !== summary && !record.inert.has(child)) { record.inert.set(child,child.inert); child.inert = true; }
        } else restoreDisclosureChildren(record);
        record.open = open; node.open = open;
        delete node.dataset.studioDisclosureAnimating;
        node.style.removeProperty('--studio-disclosure-height'); node.style.removeProperty('--studio-disclosure-gap');
        const target = node.getBoundingClientRect().height;
        if (open) record.openHeight = target; else record.closedHeight = target;
        const chrome = target - (parseFloat(getComputedStyle(node,'::details-content').blockSize) || 0);
        record.summaryGap = parseFloat(getComputedStyle(summary).marginBottom) || 0;
        node.dataset.studioDisclosureAnimating = 'true';
        if (!open) node.style.setProperty('--studio-disclosure-gap',`${gap}px`);
        move(node,open?1:0,{channel:record.progressChannel,from:record.progress,response:quickResponse,min:0,max:1,staticValue:open?1:0,
            apply:value=>{record.progress=value;node.style.setProperty('--studio-disclosure-progress',String(value));},settled:()=>node.style.removeProperty('--studio-disclosure-progress')});
        move(node,target,{from,response:panelResponse,min:record.closedHeight ?? Math.min(from,target),max:Math.max(record.openHeight ?? 0,from,target),staticValue:target,positionEpsilon:.1,velocityEpsilon:.5,
            // The anonymous content remains naturally sized during height motion.
            // Follow nested disclosures/live rows instead of freezing an old end height.
            measureTarget:open?()=>chrome+(parseFloat(getComputedStyle(node,'::details-content').blockSize)||0):null,
            apply:value => node.style.setProperty('--studio-disclosure-height',`${value}px`),
            settled:() => {
                delete node.dataset.studioDisclosureAnimating;
                node.style.removeProperty('--studio-disclosure-height'); node.style.removeProperty('--studio-disclosure-gap');
                record.height = node.getBoundingClientRect().height; restoreDisclosureChildren(record);
            }});
    }
    let composing = false;
    document.addEventListener('compositionstart', () => { composing = true; }, true);
    document.addEventListener('compositionend', () => { composing = false; }, true);
    document.addEventListener('click', event => {
        if (!detailsSupported || event.defaultPrevented || composing || !(event.target instanceof Element)) return;
        const summary = event.target.closest('summary'), node = summary?.parentElement;
        if (!(node instanceof HTMLDetailsElement) || node.firstElementChild !== summary || event.target.closest('a,button,input,select,textarea,[contenteditable]')) return;
        event.preventDefault(); animateDisclosure(node,!node.open,node.getBoundingClientRect().height);
    });
    const segments = new Map(), subpages = new Map();
    const segmentSelector = '.segmented,.workspace-tabs,.manager-filters,[data-studio-subpage-nav],.sidebar,.parameter-groups,.mode-buttons,.parameter-switch,.inertia-source-options,.graphics-mode-options,.mode-grid,[data-studio-switch-group]';
    function selectionItems(group) {
        if (group.matches('.sidebar')) return [...group.querySelectorAll('.nav-item[data-page]')];
        if (group.matches('.parameter-switch')) return [...group.querySelectorAll(':scope > .switch-value')];
        return [...group.children].filter(node=>node instanceof HTMLButtonElement || node.tagName==='LABEL'&&node.querySelector('input[type=radio]'));
    }
    function itemKey(node) {
        const input=node.querySelector('input[type=radio]');
        return input ? `${input.name}=${input.value}` : [...node.attributes].filter(attribute=>attribute.name.startsWith('data-')&&!attribute.name.startsWith('data-studio-')&&!attribute.name.startsWith('data-fixture-')).map(attribute=>`${attribute.name}=${attribute.value}`).join(',') || node.id || node.textContent.trim();
    }
    function segmentKey(group, items) {
        const profile=document.querySelector('.studio-shell-profile strong')?.textContent || '';
        if (group.matches('.parameter-switch')) return `${profile}:switch:${group.querySelector('input').dataset.profileField}`;
        // Optional actions may appear beside the choices after selection; they
        // must not replace the group's presentation state mid-flight.
        return `${profile}:${group.dataset.studioSubpageNav || group.matches('.sidebar') ? 'nav:'+ (group.dataset.studioSubpageNav||'sidebar') : group.classList[0]}:${itemKey(items[0])}`;
    }
    function selectedItem(group, items) {
        if (group.matches('.parameter-switch')) return items[group.querySelector('input').checked?0:1];
        return items.find(item=>item.matches('[aria-pressed="true"],[aria-selected="true"],[aria-current],.active,.selected') || item.querySelector('input[type=radio]:checked'));
    }
    function updateSegments() {
        for (const group of document.querySelectorAll(segmentSelector)) {
            const items=selectionItems(group),active=selectedItem(group,items);
            if (!active || items.length<2 || !group.getClientRects().length) continue;
            const key=segmentKey(group,items);
            let record=segments.get(key);
            if (!record) { record={channels:{},values:{},surfaces:new Map()};segments.set(key,record); }
            if (group.matches('.graphics-mode-options,.mode-grid')) {
                // Large cards crossfade their own surfaces; moving a solid panel
                // away from already-white selected text would make it unreadable.
                for (const item of items) {
                    const id=itemKey(item),target=item===active?1:0;
                    let surface=record.surfaces.get(id);
                    if (!surface) { surface={value:target,channel:{}};record.surfaces.set(id,surface); }
                    item.dataset.studioSelection='true';
                    move(item,target,{channel:surface.channel,from:surface.value,response:quickResponse,min:0,max:1,staticValue:target,
                        apply:value=>{surface.value=value;item.style.setProperty('--studio-selection',String(value));}});
                }
                continue;
            }
            const bounds=group.getBoundingClientRect(),selected=active.getBoundingClientRect(),style=getComputedStyle(group);
            const values={x:selected.left-bounds.left-parseFloat(style.borderLeftWidth||0)+group.scrollLeft,
                y:selected.top-bounds.top-parseFloat(style.borderTopWidth||0)+group.scrollTop,width:selected.width,height:selected.height};
            let indicator=group.querySelector(':scope > .studio-segment-indicator');
            if (!indicator) { indicator=document.createElement('span');indicator.className='studio-segment-indicator';indicator.setAttribute('aria-hidden','true');group.append(indicator); }
            group.dataset.studioSegmented='true';
            const underline=!!group.dataset.studioSubpageNav;
            for (const [axis,target] of Object.entries(values)) {
                if (underline&&(axis==='y'||axis==='height')) continue;
                record.channels[axis] ||= {};record.values[axis] ??=target;
                move(indicator,target,{channel:record.channels[axis],from:record.values[axis],response:quickResponse,min:0,max:Math.max(axis==='height'||axis==='y'?group.scrollHeight:group.scrollWidth,target),positionEpsilon:.1,velocityEpsilon:.5,
                    apply:value=>{record.values[axis]=value;if(axis==='x'||axis==='y')indicator.style.transform=`translate(${record.values.x||0}px,${underline?0:record.values.y||0}px)`;else indicator.style[axis]=`${value}px`;}});
            }
            if (group.dataset.studioSubpageNav) {
                const name=group.dataset.studioSubpageNav,panel=[...document.querySelectorAll('[data-studio-subpage-panel]')].find(node=>node.dataset.studioSubpagePanel===name);
                const selection=panel?.dataset.studioSubpageKey || active.dataset.studioSubpageKey;
                if(panel&&subpages.has(name)&&subpages.get(name)!==selection)enterPage(panel);
                subpages.set(name,selection);
            }
        }
        for(const input of document.querySelectorAll('input[role=switch]:not(.parameter-switch input)')) {
            const key=`switch:${input.id||input.getAttribute('aria-label')}`,target=input.checked?1:0;
            let record=segments.get(key);if(!record){record={value:target,channel:{}};segments.set(key,record);}
            input.dataset.studioSwitch='true';
            move(input,target,{channel:record.channel,from:record.value,response:quickResponse,min:0,max:1,staticValue:target,
                apply:value=>{record.value=value;input.style.setProperty('--studio-switch-progress',String(value));}});
        }
    }
    function enhanceInteractiveMotion() {
        if (detailsSupported) for (const node of document.querySelectorAll('details')) {
            if (node.firstElementChild?.tagName !== 'SUMMARY') continue;
            const record = rememberDisclosure(node);
            if (record.open !== node.open && motions.has(node)) animateDisclosure(node,node.open);
            // Programmatic reveal for a field/anchor stays synchronous; render
            // and telemetry refreshes do not replay disclosure entrance motion.
            else if (record.open !== node.open) { record.open=node.open; record.progress=node.open?1:0; record.height=node.getBoundingClientRect().height; restoreDisclosureChildren(record); }
            else if (!motions.has(node)) { record.height=node.getBoundingClientRect().height; record.summaryGap=parseFloat(getComputedStyle(node.firstElementChild).marginBottom)||0; }
        }
        updateSegments();
    }
    let enhancementQueued = false;
    const refreshMotion = () => {
        if (enhancementQueued) return;
        enhancementQueued = true; queueMicrotask(() => { enhancementQueued=false; enhanceInteractiveMotion(); });
    };
    new MutationObserver(records => {
        if (records.some(record=>record.type==='attributes' ? record.target instanceof HTMLDetailsElement || record.target.closest(segmentSelector)
            : [...record.addedNodes].some(node=>node instanceof Element && (node.matches(`details,${segmentSelector}`) || node.querySelector(`details,${segmentSelector}`))))) refreshMotion();
    }).observe(document.body,{childList:true,subtree:true,attributes:true,attributeFilter:['open','class','aria-pressed','aria-selected','aria-current','aria-checked','checked']});
    document.addEventListener('change',event=>{if(event.target.matches('input[type=radio],input[role=switch]'))refreshMotion();});
    window.addEventListener('resize',refreshMotion);
    document.fonts.ready.then(refreshMotion); enhanceInteractiveMotion();
    window.serialArmStudio = {enterPage,anchorPopover};
})();

// Progressive enhancement keeps the original selects and their change handlers
// as the source of truth, while matching StudioSelect's top-layer option menu
(() => {
    let sequence = 0;
    const enhanced = new WeakMap();
    const chevron = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="m5 9 7 7 7-7"/></svg>';
    function enhanceSelects() {
        document.querySelectorAll('select:not([multiple])').forEach(select => {
            const existing = enhanced.get(select);
            if (existing) { existing.update(); return; }
            const wrapper = document.createElement('div'); wrapper.className = 'studio-select';
            const trigger = document.createElement('button');
            trigger.type = 'button'; trigger.className = 'ros-select-trigger';
            trigger.setAttribute('aria-haspopup', 'listbox'); trigger.setAttribute('aria-expanded', 'false');
            const selectedText = document.createElement('span');
            trigger.append(selectedText); trigger.insertAdjacentHTML('beforeend', chevron);
            const label = select.getAttribute('aria-label') || select.labels?.[0]?.firstChild?.textContent?.trim() || select.closest('.field')?.querySelector('label')?.textContent || select.id;
            trigger.setAttribute('aria-label', label);
            const menu = document.createElement('div'); menu.className = 'studio-popover ros-select-options';
            menu.popover = 'auto'; menu.id = `studio-options-${++sequence}`;
            menu.setAttribute('role', 'listbox'); menu.setAttribute('aria-label', label);
            trigger.setAttribute('aria-controls', menu.id);
            select.before(wrapper); wrapper.append(select, trigger, menu);
            select.classList.add('studio-native-select'); select.tabIndex = -1; select.setAttribute('aria-hidden', 'true');
            select.addEventListener('click',event=>event.preventDefault());
            select.addEventListener('focus',()=>trigger.focus({preventScroll:true}));
            const update = () => {
                const text = select.selectedOptions[0]?.textContent || '';
                if (selectedText.textContent !== text) selectedText.textContent = text;
                const disabled = select.matches(':disabled');
                if (trigger.disabled !== disabled) trigger.disabled = disabled;
                for (const name of ['aria-labelledby', 'aria-describedby', 'aria-invalid']) {
                    const value = select.getAttribute(name);
                    if (value !== null) { if (trigger.getAttribute(name) !== value) trigger.setAttribute(name, value); }
                    else trigger.removeAttribute(name);
                }
                if (select.hasAttribute('aria-labelledby')) menu.setAttribute('aria-labelledby', select.getAttribute('aria-labelledby'));
                else menu.removeAttribute('aria-labelledby');
            };
            function position() {
                const rect = trigger.getBoundingClientRect();
                menu.style.width = `${Math.min(Math.max(rect.width, 250), innerWidth - 16)}px`;
                const height = Math.min(360, innerHeight - 16);
                menu.style.maxHeight = `${height}px`;
                const below = Math.max(0,innerHeight - rect.bottom - 8), above = Math.max(0,rect.top - 8);
                const useBelow = below >= Math.min(height,menu.scrollHeight) || below >= above;
                menu.style.maxHeight = `${Math.min(height,useBelow?below:above)}px`;
                menu.style.left = `${Math.max(8, Math.min(rect.left, innerWidth - menu.offsetWidth - 8))}px`;
                menu.style.top = useBelow
                    ? `${rect.bottom + 5}px` : `${Math.max(8, rect.top - menu.offsetHeight - 5)}px`;
            }
            function prepareOptions() {
                update();
                menu.replaceChildren();
                const searchable = select.options.length > 8;
                let search;
                if (searchable) {
                    search = document.createElement('input'); search.type = 'search';
                    search.className = 'studio-select-search';
                    search.placeholder = document.documentElement.lang === 'en' ? 'Search options' : '搜索选项';
                    search.setAttribute('aria-label', search.placeholder); menu.append(search);
                }
                const scroll = document.createElement('div'); scroll.className = 'ros-select-scroll'; menu.append(scroll);
                for (const option of select.options) {
                    if (option.hidden) continue;
                    const button = document.createElement('button'); button.type = 'button'; button.setAttribute('role', 'option');
                    button.setAttribute('aria-selected', String(option.selected)); button.dataset.value = option.value;
                    button.disabled = option.disabled || option.parentElement?.disabled === true;
                    const text = document.createElement('strong'); text.textContent = option.textContent;
                    const check = document.createElement('span'); check.setAttribute('aria-hidden', 'true'); check.textContent = option.selected ? '✓' : '';
                    button.append(text, check);
                    button.onclick = () => {
                        if (select.value !== option.value) {
                            select.value = option.value;
                            select.dispatchEvent(new Event('input', {bubbles: true}));
                            select.dispatchEvent(new Event('change', {bubbles: true}));
                        }
                        if (menu.isConnected && menu.matches(':popover-open')) menu.hidePopover();
                        if (trigger.isConnected) { update(); trigger.focus(); }
                    };
                    scroll.append(button);
                }
                if (search) search.oninput = () => {
                    for (const option of scroll.children) option.hidden = !option.textContent.toLowerCase().includes(search.value.toLowerCase());
                    position();
                };
            }
            // The browser treats the invoker as part of its popover. A manual
            // onclick toggle races auto light-dismiss on a second pointer click
            trigger.popoverTargetElement = menu;
            trigger.popoverTargetAction = 'toggle';
            let openingKey = '';
            menu.addEventListener('beforetoggle', event => {
                if (event.newState === 'open') prepareOptions();
            });
            trigger.onkeydown = event => {
                if (event.isComposing || !['ArrowDown', 'ArrowUp'].includes(event.key)) return;
                event.preventDefault();
                if (!menu.matches(':popover-open')) { openingKey=event.key; trigger.click(); }
                else menu.querySelector('[aria-selected="true"]:not(:disabled),button:not(:disabled)')?.focus();
            };
            menu.onkeydown = event => {
                if (event.isComposing) return;
                const options = [...menu.querySelectorAll('[role="option"]')].filter(option => !option.disabled && !option.hidden);
                const current = options.indexOf(document.activeElement);
                if (['ArrowDown', 'ArrowUp', 'Home', 'End'].includes(event.key) && options.length) {
                    event.preventDefault();
                    const index = event.key === 'Home' ? 0 : event.key === 'End' ? options.length - 1 : current < 0 ? (event.key==='ArrowUp'?options.length-1:0)
                        : (current + (event.key === 'ArrowDown' ? 1 : -1) + options.length) % options.length;
                    options[index].focus();
                }
            };
            menu.addEventListener('toggle', event => {
                const open = event.newState === 'open';
                trigger.setAttribute('aria-expanded', String(open));
                if (open) {
                    position();
                    const options=[...menu.querySelectorAll('[role=option]:not(:disabled)')];
                    (openingKey==='ArrowUp'?options.at(-1):openingKey?menu.querySelector('[aria-selected="true"]:not(:disabled)')||options[0]:menu.querySelector('input')||menu.querySelector('[aria-selected="true"]:not(:disabled)')||options[0])?.focus();
                    openingKey='';
                } else if (menu.contains(document.activeElement)) trigger.focus();
            });
            window.serialArmStudio.anchorPopover(menu, trigger);
            select.addEventListener('change', update);
            for (const label of select.labels || []) label.addEventListener('click', event => {
                const interactive=event.target instanceof Element?event.target.closest('button,input,a,textarea,[contenteditable]'):null;
                if (!interactive || interactive===select) { event.preventDefault(); trigger.focus({preventScroll:true}); }
            });
            enhanced.set(select, {update}); update();
        });
    }
    let queued = false;
    const observer = new MutationObserver(records => {
        const relevant = records.some(record => record.target instanceof HTMLSelectElement || [...record.addedNodes].some(node => node instanceof Element && (node.matches('select') || node.querySelector('select'))));
        if (!relevant || queued) return;
        queued = true; queueMicrotask(() => { queued = false; enhanceSelects(); });
    });
    observer.observe(document.body, {childList: true, subtree: true, attributes: true, attributeFilter: ['disabled', 'aria-labelledby', 'aria-describedby', 'aria-invalid']});
    const closeMenus = event => {
        for (const menu of document.querySelectorAll('.ros-select-options:popover-open')) {
            if (!menu.contains(event.target)) menu.hidePopover();
        }
    };
    window.addEventListener('resize', closeMenus);
    document.addEventListener('scroll', closeMenus, true);
    window.serialArmStudio = {...window.serialArmStudio,enhanceSelects};
})();
