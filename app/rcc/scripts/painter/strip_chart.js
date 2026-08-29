// Scrolling strip chart.
//
// Each dataset gets its own coloured trace on a shared time axis. History
// grows until the configured length, then scrolls. Demonstrates ring-buffer
// state, per-trace colouring, and a unified Y-axis derived from every
// dataset's min/max.

const HISTORY = 240;

const traces = [];

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

function paint(ctx, w, h) {
  // Cream paper background + vignette.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  // Header strip.
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("STRIP  CHART", 14, 18);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(datasets.length + " channels", w - 14, 18);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 22, w - 28, 1);

  // Plot card.
  const padL = 50;
  const padR = 14;
  const padT = 32;
  const padB = 38;
  const plotX = padL;
  const plotY = padT;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(plotX + 1, plotY + 2, plotW, plotH);
  ctx.fillStyle = theme.alternate_base;
  ctx.fillRect(plotX, plotY, plotW, plotH);
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.strokeRect(plotX + 0.5, plotY + 0.5, plotW - 1, plotH - 1);

  if (datasets.length === 0) return;

  // Unified Y range across every dataset.
  let lo = Number.POSITIVE_INFINITY;
  let hi = Number.NEGATIVE_INFINITY;
  for (let i = 0; i < datasets.length; ++i) {
    lo = Math.min(lo, datasets[i].min);
    hi = Math.max(hi, datasets[i].max);
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi) || hi <= lo) {
    lo = 0;
    hi = 100;
  }
  const span = hi - lo;

  // Horizontal grid lines + Y labels.
  ctx.strokeStyle = theme.widget_border;
  ctx.fillStyle    = theme.placeholder_text;
  ctx.font         = "9px sans-serif";
  ctx.textBaseline = "middle";
  ctx.textAlign    = "right";
  for (let i = 0; i <= 4; ++i) {
    const y = plotY + (plotH / 4) * i;
    if (i > 0 && i < 4) {
      ctx.beginPath();
      ctx.moveTo(plotX, y);
      ctx.lineTo(plotX + plotW, y);
      ctx.stroke();
    }
    const v = hi - (span / 4) * i;
    ctx.fillText(v.toFixed(1), plotX - 6, y);
  }
  ctx.textBaseline = "alphabetic";
  ctx.textAlign    = "start";

  // Vertical guides at quarter intervals.
  ctx.strokeStyle = theme.widget_border;
  for (let i = 1; i < 4; ++i) {
    const x = plotX + (plotW / 4) * i;
    ctx.beginPath();
    ctx.moveTo(x, plotY);
    ctx.lineTo(x, plotY + plotH);
    ctx.stroke();
  }

  // Each trace.
  for (let i = 0; i < traces.length; ++i) {
    const t = traces[i];
    if (t.length < 2) continue;

    const color = theme.widget_colors[i % theme.widget_colors.length];

    ctx.strokeStyle = color;
    ctx.lineWidth   = 1.8;
    ctx.beginPath();
    for (let k = 0; k < t.length; ++k) {
      const x = plotX + (k / (HISTORY - 1)) * plotW;
      const n = (t[k] - lo) / span;
      const y = plotY + plotH - Math.max(0, Math.min(1, n)) * plotH;
      if (k === 0) ctx.moveTo(x, y);
      else         ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Dot at the latest sample.
    const lastN = (t[t.length - 1] - lo) / span;
    const lastX = plotX + ((t.length - 1) / (HISTORY - 1)) * plotW;
    const lastY = plotY + plotH - Math.max(0, Math.min(1, lastN)) * plotH;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(lastX + 3, lastY);
    ctx.arc(lastX, lastY, 3, 0, Math.PI * 2);
    ctx.fill();
  }

  // Legend pills along the bottom.
  let lx = plotX;
  ctx.font = "10px sans-serif";
  for (let i = 0; i < datasets.length; ++i) {
    const text   = datasets[i].title || ("CH " + (i + 1));
    const color  = theme.widget_colors[i % theme.widget_colors.length];
    const tw     = ctx.measureTextWidth(text);
    const pillW  = 14 + tw + 14;
    if (lx + pillW > plotX + plotW) break;

    // Swatch.
    ctx.fillStyle = color;
    ctx.fillRect(lx, h - padB + 12, 8, 8);
    // Label.
    ctx.fillStyle = theme.placeholder_text;
    ctx.fillText(text, lx + 12, h - padB + 19);

    lx += pillW;
  }
}
