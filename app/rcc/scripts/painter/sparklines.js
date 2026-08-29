// Sparkline grid.
//
// One miniature line chart per dataset, laid out as cards in a grid. Each
// card shows the dataset title, current value, and a 60-sample history
// trace shaded under the line. Trend arrow indicates short-term direction.

const HISTORY = 60;
const traces  = [];

function onFrame() {
  while (traces.length < datasets.length) traces.push([]);

  for (let i = 0; i < datasets.length; ++i) {
    const v = datasets[i].value;
    if (Number.isFinite(v)) {
      traces[i].push(v);
      if (traces[i].length > HISTORY) traces[i].shift();
    }
  }
}

function trend_dir(t) {
  if (!t || t.length < 8) return 0;
  const tail = t.slice(-8);
  const a    = tail[0];
  const b    = tail[tail.length - 1];
  const span = Math.max(...tail) - Math.min(...tail);
  if (span === 0) return 0;
  const delta = (b - a) / span;
  if (delta > 0.15) return 1;
  if (delta < -0.15) return -1;
  return 0;
}

function paint(ctx, w, h) {
  // Cream paper background.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  if (datasets.length === 0) return;

  const cols = Math.max(1, Math.ceil(Math.sqrt(datasets.length * w / h)));
  const rows = Math.ceil(datasets.length / cols);
  const padX = 12;
  const padY = 12;
  const gap  = 8;
  const cw   = (w - padX * 2 - gap * (cols - 1)) / cols;
  const ch   = (h - padY * 2 - gap * (rows - 1)) / rows;

  for (let i = 0; i < datasets.length; ++i) {
    const r  = Math.floor(i / cols);
    const c  = i % cols;
    const x  = padX + c * (cw + gap);
    const y  = padY + r * (ch + gap);
    const ds = datasets[i];
    const traceColor = theme.widget_colors[i % theme.widget_colors.length];

    // Card with shadow.
    ctx.fillStyle = theme.widget_border;
    ctx.fillRect(x + 1, y + 2, cw, ch);
    ctx.fillStyle = theme.alternate_base;
    ctx.fillRect(x, y, cw, ch);
    ctx.strokeStyle = theme.widget_border;
    ctx.lineWidth = 1;
    ctx.strokeRect(x + 0.5, y + 0.5, cw - 1, ch - 1);

    // Title.
    ctx.fillStyle    = theme.placeholder_text;
    ctx.font         = "bold 10px sans-serif";
    ctx.textAlign    = "start";
    ctx.textBaseline = "alphabetic";
    ctx.fillText((ds.title || "").substring(0, 22), x + 10, y + 16);

    // Trend arrow on the right.
    const dir = trend_dir(traces[i]);
    if (dir !== 0) {
      const ax = x + cw - 14;
      const ay = y + 14;
      ctx.fillStyle = dir > 0 ? theme.widget_highlight : theme.alarm;
      ctx.beginPath();
      if (dir > 0) {
        ctx.moveTo(ax, ay - 4);
        ctx.lineTo(ax + 5, ay + 3);
        ctx.lineTo(ax - 5, ay + 3);
      } else {
        ctx.moveTo(ax, ay + 4);
        ctx.lineTo(ax + 5, ay - 3);
        ctx.lineTo(ax - 5, ay - 3);
      }
      ctx.closePath();
      ctx.fill();
    }

    // Current value (large).
    ctx.fillStyle = theme.widget_text;
    ctx.font      = "bold 18px sans-serif";
    const valTxt  = Number.isFinite(ds.value) ? ds.value.toFixed(2) : "--";
    ctx.fillText(valTxt, x + 10, y + 38);

    if (ds.units) {
      const valW = ctx.measureTextWidth(valTxt);
      ctx.fillStyle = theme.placeholder_text;
      ctx.font      = "10px sans-serif";
      ctx.fillText(ds.units, x + 14 + valW, y + 38);
    }

    // Sparkline trace, filled under the line.
    const t = traces[i];
    if (t && t.length >= 2) {
      const span = (ds.max - ds.min) || 1;
      const px = x + 8;
      const pw = cw - 16;
      const ph = ch - 50;
      const py = y + ch - 6;
      const top = py - ph;

      // Fill under the line.
      ctx.fillStyle = traceColor;
      ctx.globalAlpha = 0.18;
      ctx.beginPath();
      ctx.moveTo(px, py);
      for (let k = 0; k < t.length; ++k) {
        const xk = px + (k / (HISTORY - 1)) * pw;
        const n  = (t[k] - ds.min) / span;
        const yk = py - Math.max(0, Math.min(1, n)) * ph;
        ctx.lineTo(xk, yk);
      }
      ctx.lineTo(px + (t.length - 1) / (HISTORY - 1) * pw, py);
      ctx.closePath();
      ctx.fill();
      ctx.globalAlpha = 1;

      // Line on top.
      ctx.strokeStyle = traceColor;
      ctx.lineWidth   = 1.5;
      ctx.beginPath();
      for (let k = 0; k < t.length; ++k) {
        const xk = px + (k / (HISTORY - 1)) * pw;
        const n  = (t[k] - ds.min) / span;
        const yk = py - Math.max(0, Math.min(1, n)) * ph;
        if (k === 0) ctx.moveTo(xk, yk);
        else         ctx.lineTo(xk, yk);
      }
      ctx.stroke();

      // Latest-value dot.
      const lastN  = (t[t.length - 1] - ds.min) / span;
      const lastY  = py - Math.max(0, Math.min(1, lastN)) * ph;
      const lastX  = px + ((t.length - 1) / (HISTORY - 1)) * pw;
      ctx.fillStyle = traceColor;
      ctx.beginPath();
      ctx.moveTo(lastX + 3, lastY);
      ctx.arc(lastX, lastY, 3, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  ctx.textBaseline = "alphabetic";
  ctx.textAlign    = "start";
}
