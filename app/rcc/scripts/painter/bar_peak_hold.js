// Vertical bars with peak-hold markers.
//
// One bar per dataset, rendered as a stack of LED-style segments. The most
// recent peak per channel is held briefly then decays linearly so spikes
// remain visible without freezing.

const SEGMENTS    = 24;
const SEG_GAP     = 2;
const HOLD_MS     = 1200;
const DECAY_PER_S = 0.6;

const peaks  = [];
const peakTs = [];
let lastTs   = 0;

function clamp01(x) { return Math.max(0, Math.min(1, x)); }

function lit_color(t) {
  if (t < 0.66) return theme.widget_highlight;
  if (t < 0.85) return theme.accent;
  return theme.alarm;
}

function unlit_color(t) {
  return theme.widget_border;
}

function onFrame() {
  const now = frame.timestampMs;
  const dt  = lastTs > 0 ? Math.max(0, (now - lastTs) * 1e-3) : 0;
  lastTs    = now;

  while (peaks.length < datasets.length) {
    peaks.push(0);
    peakTs.push(0);
  }

  for (let i = 0; i < datasets.length; ++i) {
    const ds   = datasets[i];
    const span = (ds.max - ds.min) || 1;
    const v    = Number.isFinite(ds.value) ? ds.value : ds.min;
    const norm = clamp01((v - ds.min) / span);
    if (norm > peaks[i]) {
      peaks[i]  = norm;
      peakTs[i] = now;
    } else if (now - peakTs[i] > HOLD_MS) {
      peaks[i] = Math.max(0, peaks[i] - DECAY_PER_S * dt);
    }
  }
}

function draw_vbar(ctx, x, y, w, h, level, peak) {
  ctx.fillStyle = theme.alternate_base;
  ctx.fillRect(x, y, w, h);

  const segH = (h - SEG_GAP * (SEGMENTS - 1)) / SEGMENTS;
  for (let i = 0; i < SEGMENTS; ++i) {
    const t   = (i + 0.5) / SEGMENTS;
    const lit = t <= level;
    ctx.fillStyle = lit ? lit_color(t) : unlit_color(t);
    const sy = y + h - (i + 1) * segH - i * SEG_GAP;
    ctx.fillRect(x + 2, sy, w - 4, segH);
  }

  if (peak > 0) {
    const py = y + h - peak * h;
    ctx.fillStyle = theme.widget_border;
    ctx.fillRect(x - 2, py - 2, w + 4, 1);
    ctx.fillRect(x - 2, py + 2, w + 4, 1);
    ctx.fillStyle = theme.accent;
    ctx.fillRect(x - 3, py - 1, w + 6, 2);
  }

  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
}

function paint(ctx, w, h) {
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  if (datasets.length === 0) return;

  const padX   = 14;
  const padTop = 24;
  const padBot = 36;
  const cellW  = (w - padX * 2) / datasets.length;
  const barW   = Math.min(40, cellW * 0.55);
  const top    = padTop;
  const bot    = h - padBot;

  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("BARS", padX, 16);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(padX, 20, w - padX * 2, 1);

  for (let i = 0; i < datasets.length; ++i) {
    const ds   = datasets[i];
    const span = (ds.max - ds.min) || 1;
    const v    = Number.isFinite(ds.value) ? ds.value : ds.min;
    const norm = clamp01((v - ds.min) / span);
    const x    = padX + i * cellW + (cellW - barW) * 0.5;

    draw_vbar(ctx, x, top, barW, bot - top, norm, peaks[i] || 0);

    ctx.fillStyle = theme.placeholder_text;
    ctx.font      = "10px sans-serif";
    ctx.textAlign = "center";
    const title   = (ds.title || "").substring(0, 14);
    ctx.fillText(title, x + barW * 0.5, h - padBot + 14);

    ctx.fillStyle = theme.widget_text;
    ctx.font      = "bold 11px sans-serif";
    const txt     = Number.isFinite(ds.value) ? ds.value.toFixed(1) : "--";
    ctx.fillText(txt, x + barW * 0.5, h - padBot + 28);
  }

  ctx.textAlign = "start";
}
