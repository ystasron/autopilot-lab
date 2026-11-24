/* About sheet — a dialog that can be dismissed by button, scrim click, Escape,
   or by dragging the grabber down on touch. Kept separate from the simulation:
   js/sim.js owns the canvas, this file owns the overlay. */
(() => {
  const modal = document.getElementById('intro');
  const sheet = document.getElementById('intro-sheet');
  const grab = document.getElementById('intro-grab');
  const cta = document.getElementById('intro-close');
  const scrim = modal.querySelector('[data-close]');

  let previous = null;
  let open = false;

  const focusables = () => [...sheet.querySelectorAll(
    'button, select, input, a[href], [tabindex]:not([tabindex="-1"])'
  )].filter(el => el.offsetParent !== null);

  function show() {
    if (open) return;
    previous = document.activeElement;
    open = true;
    modal.classList.add('on');
    document.documentElement.style.overflow = 'hidden';
    requestAnimationFrame(() => cta.focus({ preventScroll: true }));
  }

  function hide() {
    if (!open) return;
    open = false;
    modal.classList.remove('on');
    document.documentElement.style.overflow = '';
    if (previous && previous.focus) previous.focus({ preventScroll: true });
  }

  cta.addEventListener('click', hide);
  scrim.addEventListener('click', hide);

  addEventListener('keydown', event => {
    if (!open) return;
    if (event.key === 'Escape') { hide(); return; }
    if (event.key !== 'Tab') return;
    const items = focusables();
    if (!items.length) return;
    const first = items[0];
    const last = items[items.length - 1];
    if (event.shiftKey && document.activeElement === first) {
      event.preventDefault();
      last.focus();
    } else if (!event.shiftKey && document.activeElement === last) {
      event.preventDefault();
      first.focus();
    }
  });

  /* drag the sheet down to dismiss (touch, narrow screens only) */
  const wide = () => matchMedia('(min-width:700px)').matches;
  let startY = 0;
  let dragY = 0;
  let dragging = false;

  grab.addEventListener('touchstart', event => {
    if (wide() || sheet.scrollTop > 0) return;
    startY = event.touches[0].clientY;
    dragY = 0;
    dragging = true;
  }, { passive: true });

  grab.addEventListener('touchmove', event => {
    if (!dragging) return;
    dragY = Math.max(0, event.touches[0].clientY - startY);
    grab.style.transition = 'none';
    grab.style.transform = `translateY(${dragY}px)`;
  }, { passive: true });

  const endDrag = () => {
    if (!dragging) return;
    dragging = false;
    grab.style.transition = '';
    grab.style.transform = '';
    if (dragY > 110) hide();
  };

  grab.addEventListener('touchend', endDrag);
  grab.addEventListener('touchcancel', endDrag);

  const later = () => setTimeout(show, 450);
  if (document.readyState === 'complete') later();
  else addEventListener('load', later);
})();
