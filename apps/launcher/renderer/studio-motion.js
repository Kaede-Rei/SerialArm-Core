'use strict';
// Retarget the current presentation value and velocity; no action waits for motion.
(() => {
    const reduced = matchMedia('(prefers-reduced-motion: reduce)');
    const motionReduced = () => reduced.matches || document.documentElement.dataset.reduceMotion === 'true';
    const motions = new Map(), anchors = new WeakMap();
    let pressed = null, frame = 0, previous = 0;
    function tick(now) {
        if (motionReduced()) { resetMotion(); return; }
        const dt = Math.max(0,Math.min((now - previous) / 1000 || 1 / 60, .032));
        previous = now;
        for (const [node, motion] of motions) {
            if (!node.isConnected) { motions.delete(node); continue; }
            // Exact critically damped step avoids frame-rate dependent overshoot.
            const omega = 2 * Math.PI / motion.response;
            const offset = motion.value - motion.target, carry = motion.velocity + omega * offset;
            const decay = Math.exp(-omega * dt);
            motion.value = motion.target + (offset + carry * dt) * decay;
            motion.velocity = (motion.velocity - omega * carry * dt) * decay;
            const bounded = Math.max(motion.min, Math.min(motion.max, motion.value));
            if (bounded !== motion.value) { motion.value = bounded; motion.velocity = 0; }
            motion.apply(motion.value);
            if (Math.abs(motion.value - motion.target) < .0001 && Math.abs(motion.velocity) < .001) {
                motion.apply(motion.target); motion.settled?.(motion.target); motions.delete(node);
            }
        }
        frame = motions.size ? requestAnimationFrame(tick) : 0;
    }
    function move(node, target, options) {
        if (motionReduced()) { options.apply(options.staticValue ?? target); options.settled?.(options.staticValue ?? target); return; }
        const motion = motions.get(node) || {value:options.from,velocity:0};
        Object.assign(motion, options, {target}); motions.set(node, motion);
        motion.apply(motion.value);
        if (frame) cancelAnimationFrame(frame);
        previous = performance.now(); frame = requestAnimationFrame(tick);
    }
    function respond(button, target) {
        move(button, target, {from:Number(getComputedStyle(button).scale) || 1,response:.18,min:.97,max:1,staticValue:1,
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
        move(page, 1, {from:current?.value ?? .94,response:.16,min:0,max:1,staticValue:1,
            apply:value => { page.style.opacity = String(value); },
            settled:() => page.style.removeProperty('opacity')});
    }
    function anchorPopover(menu, trigger) {
        const bound = anchors.get(menu);
        if (bound) { bound.trigger = trigger; return; }
        const anchor = {trigger}; anchors.set(menu, anchor);
        function animate(open) {
            const source = anchor.trigger.getBoundingClientRect();
            const left = parseFloat(menu.style.left) || menu.getBoundingClientRect().left;
            const top = parseFloat(menu.style.top) || menu.getBoundingClientRect().top;
            const above = top + menu.offsetHeight <= source.top + 1;
            const origin = Math.max(0,Math.min(menu.offsetWidth,source.left + source.width / 2 - left));
            menu.style.transformOrigin = `${origin}px ${above ? '100%' : '0%'}`;
            menu.dataset.motionPlacement = above ? 'above' : 'below';
            move(menu, open ? 1 : 0, {from:motions.get(menu)?.value ?? (open ? 0 : 1),response:.14,min:0,max:1,
                apply:value => { menu.style.opacity=String(value); menu.style.transform=motionReduced()?'none':`translateY(${(above ? 4 : -4)*(1-value)}px) scale(${.97+.03*value})`; }});
        }
        menu.addEventListener('beforetoggle', event => {
            const open = event.newState === 'open'; menu.inert = !open;
            if (open) { menu.style.removeProperty('pointer-events'); menu.style.opacity=String(motions.get(menu)?.value ?? 0); }
            else { menu.style.pointerEvents='none'; animate(false); if (menu.contains(document.activeElement)) anchor.trigger.focus({preventScroll:true}); }
        });
        menu.addEventListener('toggle', event => { if (event.newState === 'open') animate(true); });
    }
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
                const height = Math.min(360, innerHeight - 24);
                const below = innerHeight - rect.bottom - 8, above = rect.top - 8;
                const room = below >= Math.min(height, menu.scrollHeight) || below >= above ? below : above;
                menu.style.width = `${Math.min(Math.max(rect.width, 250), innerWidth - 16)}px`;
                menu.style.maxHeight = `${Math.max(70, Math.min(height, room))}px`;
                menu.style.left = `${Math.max(8, Math.min(rect.left, innerWidth - menu.offsetWidth - 8))}px`;
                menu.style.top = below >= Math.min(height, menu.scrollHeight) || below >= above
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
                };
            }
            // The browser treats the invoker as part of its popover. A manual
            // onclick toggle races auto light-dismiss on a second pointer click
            trigger.popoverTargetElement = menu;
            trigger.popoverTargetAction = 'toggle';
            menu.addEventListener('beforetoggle', event => {
                if (event.newState === 'open') prepareOptions();
            });
            trigger.onkeydown = event => {
                if (event.isComposing || !['ArrowDown', 'ArrowUp'].includes(event.key)) return;
                event.preventDefault();
                if (!menu.matches(':popover-open')) trigger.click();
                else menu.querySelector('[aria-selected="true"]:not(:disabled),button:not(:disabled)')?.focus();
            };
            menu.onkeydown = event => {
                if (event.isComposing) return;
                const options = [...menu.querySelectorAll('[role="option"]')].filter(option => !option.disabled && !option.hidden);
                const current = options.indexOf(document.activeElement);
                if (['ArrowDown', 'ArrowUp', 'Home', 'End'].includes(event.key) && options.length) {
                    event.preventDefault();
                    const index = event.key === 'Home' ? 0 : event.key === 'End' ? options.length - 1
                        : (current + (event.key === 'ArrowDown' ? 1 : -1) + options.length) % options.length;
                    options[index].focus();
                }
            };
            menu.addEventListener('toggle', event => {
                const open = event.newState === 'open';
                trigger.setAttribute('aria-expanded', String(open));
                if (open) {
                    position();
                    (menu.querySelector('input') || menu.querySelector('[aria-selected="true"]:not(:disabled)') || menu.querySelector('button:not(:disabled)'))?.focus();
                } else if (menu.contains(document.activeElement)) trigger.focus();
            });
            window.serialArmStudio.anchorPopover(menu, trigger);
            select.addEventListener('change', update);
            for (const label of select.labels || []) label.addEventListener('click', event => {
                if (event.target === label) { event.preventDefault(); trigger.focus(); }
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
