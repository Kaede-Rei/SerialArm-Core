'use strict';
// Same quick spring as AT-System-Integration's StudioMotion: k=900, c=50, m=.7.
// Retain velocity when a press reverses, without introducing a renderer framework.
(() => {
    const reduced = matchMedia('(prefers-reduced-motion: reduce)');
    const springs = new Map();
    let pressed = null, frame = 0, previous = 0;
    function tick(now) {
        let remaining = Math.min((now - previous) / 1000 || 1 / 60, .032);
        previous = now;
        while (remaining > 0) {
            const dt = Math.min(remaining, .004);
            for (const spring of springs.values()) {
                spring.velocity += ((900 * (spring.target - spring.value) - 50 * spring.velocity) / .7) * dt;
                spring.value += spring.velocity * dt;
            }
            remaining -= dt;
        }
        for (const [button, spring] of springs) {
            if (!button.isConnected || reduced.matches) { button.style.removeProperty('scale'); springs.delete(button); continue; }
            button.style.scale = String(spring.value);
            if (Math.abs(spring.value - spring.target) < .0001 && Math.abs(spring.velocity) < .001) {
                if (spring.target === 1) button.style.removeProperty('scale');
                else button.style.scale = String(spring.target);
                springs.delete(button);
            }
        }
        frame = springs.size ? requestAnimationFrame(tick) : 0;
    }
    function respond(button, target) {
        if (reduced.matches) return;
        const spring = springs.get(button) || { value: Number(button.style.scale) || 1, velocity: 0 };
        spring.target = target; springs.set(button, spring);
        if (!frame) { previous = performance.now(); frame = requestAnimationFrame(tick); }
    }
    function release() { if (pressed?.isConnected) respond(pressed, 1); pressed = null; }
    function press(event) {
        if (reduced.matches || event instanceof PointerEvent && event.button !== 0) return;
        if (event instanceof KeyboardEvent && (event.repeat || ![' ', 'Enter'].includes(event.key))) return;
        const button = event.target instanceof Element ? event.target.closest('button') : null;
        if (!button || button.disabled || button.closest('[inert]') || button.dataset.motion === 'none') return;
        if (event instanceof KeyboardEvent && event.target !== button) return;
        release(); pressed = button; respond(button, button.clientWidth > 160 ? .993 : .973);
    }
    document.addEventListener('pointerdown', press, true);
    document.addEventListener('pointerup', release, true);
    document.addEventListener('pointercancel', release, true);
    document.addEventListener('keydown', press, true);
    document.addEventListener('keyup', event => { if ([' ', 'Enter'].includes(event.key)) release(); }, true);
    window.addEventListener('blur', release);
    reduced.addEventListener('change', () => {
        release();
        if (reduced.matches) { for (const button of springs.keys()) button.style.removeProperty('scale'); springs.clear(); }
    });
})();

// Progressive enhancement keeps the original selects and their change handlers
// as the source of truth, while matching StudioSelect's top-layer option menu.
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
            // onclick toggle races auto light-dismiss on a second pointer click.
            trigger.popoverTargetElement = menu;
            trigger.popoverTargetAction = 'toggle';
            menu.addEventListener('beforetoggle', event => {
                if (event.newState === 'open') prepareOptions();
            });
            trigger.onkeydown = event => {
                if (!['ArrowDown', 'ArrowUp'].includes(event.key)) return;
                event.preventDefault();
                if (!menu.matches(':popover-open')) trigger.click();
                else menu.querySelector('[aria-selected="true"]:not(:disabled),button:not(:disabled)')?.focus();
            };
            menu.onkeydown = event => {
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
    observer.observe(document.body, {childList: true, subtree: true, attributes: true, attributeFilter: ['disabled']});
    const closeMenus = event => {
        for (const menu of document.querySelectorAll('.ros-select-options:popover-open')) {
            if (!menu.contains(event.target)) menu.hidePopover();
        }
    };
    window.addEventListener('resize', closeMenus);
    document.addEventListener('scroll', closeMenus, true);
    window.serialArmStudio = {enhanceSelects};
})();

// Workspace pages scroll as one document, including over their sticky model.
// Explicit Ctrl/Meta wheel remains available for camera zoom; Model Viewer
// keeps its ordinary wheel zoom because it is not in a workbench layout.
document.addEventListener('wheel', event => {
    if (event.ctrlKey || event.metaKey || !(event.target instanceof Element)) return;
    if (event.target.closest('.workbench-layout #workbench-viewport')) event.stopPropagation();
}, {capture: true, passive: true});
