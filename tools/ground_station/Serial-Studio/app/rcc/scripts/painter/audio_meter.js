// Audio-style VU meter with segmented bars, peak hold, and dB scale.
//
// Each dataset becomes a horizontal bar broken into LED-style segments
// (green / amber / crimson by zone). The active level is drawn in saturated
// colours; idle segments are drawn in soft pastels so the full meter range
// is always visible. A peak-hold marker lingers above each bar and decays
// after a short hold time.

const HOLD_MS   = 1400;
const FALL_DB_S = 18;
const SEGMENTS  = 32;
const SEG_GAP   = 2;
const T_NOMINAL = 0.70;     // green up to here
const T_HOT     = 0.90;     // amber up to here, crimson above

const peaks = [];
const peakAt = [];
let lastTs = 0;

function lit_color(t) {
  if (t < T_NOMINAL) return theme.widget_highlight;
  if (t < T_HOT)     return theme.accent;
  return theme.alarm;
}

function unlit_color(t) {
  return theme.widget_border;
}

function clamp01(x) { return Math.max(0, Math.min(1, x)); }

function onFrame() {
  const now = frame.timestampMs;
  const dt  = lastTs > 0 ? Math.max(0, (now - lastTs) * 1e-3) : 0;
  lastTs    = now;

  while (peaks.length < datasets.length) {
    peaks.push(0);
    peakAt.push(0);
  }

  for (let i = 0; i < datasets.length; ++i) {
    const ds   = datasets[i];
    const span = (ds.max - ds.min) || 1;
    const norm = Number.isFinite(ds.value)
      ? clamp01((ds.value - ds.min) / span)
      : 0;
    if (norm > peaks[i]) {
      peaks[i] = norm;
      peakAt[i] = now;
    } else if (now - peakAt[i] > HOLD_MS) {
      peaks[i] = Math.max(0, peaks[i] - (FALL_DB_S / 66) * dt);
    }
  }
}

function draw_bar(ctx, x, y, w, h, level, peak) {
  // Recessed track.
  ctx.fillStyle = theme.alternate_base;
  ctx.fillRect(x, y, w, h);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(x, y, w, 1);
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(x, y + h - 1, w, 1);

  // Segmented LED-style bar.
  const segW = (w - SEG_GAP * (SEGMENTS - 1)) / SEGMENTS;
  for (let i = 0; i < SEGMENTS; ++i) {
    const t   = (i + 0.5) / SEGMENTS;
    const lit = t <= level;
    ctx.fillStyle = lit ? lit_color(t) : unlit_color(t);
    ctx.fillRect(x + i * (segW + SEG_GAP), y + 2, segW, h - 4);
  }

  // Peak hold marker with a faint halo.
  if (peak > 0) {
    const px = x + w * peak;
    ctx.fillStyle = theme.widget_border;
    ctx.fillRect(px - 2, y - 2, 1, h + 4);
    ctx.fillRect(px + 2, y - 2, 1, h + 4);
    ctx.fillStyle = theme.accent;
    ctx.fillRect(px - 1, y - 3, 2, h + 6);
  }

  // Outer frame.
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
}

function paint(ctx, w, h) {
  // Cream paper background.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  if (datasets.length === 0) return;

  // Card background.
  const pad = 14;
  ctx.fillStyle = theme.alternate_base;
  ctx.fillRect(pad, pad, w - pad * 2, h - pad * 2);
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.strokeRect(pad + 0.5, pad + 0.5, w - pad * 2 - 1, h - pad * 2 - 1);

  // Header strip.
  const headerY = pad + 10;
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("LEVEL  METER", pad + 16, headerY + 4);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "9px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(datasets.length === 1 ? datasets[0].title : "MULTI",
               w - pad - 16, headerY + 4);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(pad + 12, headerY + 12, w - (pad + 12) * 2, 1);

  // Bars geometry.
  const labelW = 28;
  const valueW = 64;
  const barX   = pad + 12 + labelW;
  const barW   = w - pad * 2 - 24 - labelW - valueW;
  const barH   = 16;

  // Vertically distribute bars.
  const top    = headerY + 24;
  const bot    = h - pad - 16;
  const gap    = 6;
  const total  = datasets.length * barH + (datasets.length - 1) * gap;
  let          y = top + Math.max(0, ((bot - top) - total) * 0.5);

  for (let i = 0; i < datasets.length; ++i) {
    const ds   = datasets[i];
    const span = (ds.max - ds.min) || 1;
    const norm = Number.isFinite(ds.value)
      ? clamp01((ds.value - ds.min) / span)
      : 0;

    // Channel label on the left.
    ctx.fillStyle = theme.widget_text;
    ctx.font = "bold 12px sans-serif";
    ctx.textAlign = "center";
    const tag = ds.title ? ds.title.substring(0, 2).toUpperCase() : "" + (i + 1);
    ctx.fillText(tag, pad + 12 + labelW * 0.5, y + barH * 0.5 + 4);

    draw_bar(ctx, barX, y, barW, barH, norm, peaks[i] || 0);

    // Numeric readout on the right.
    ctx.fillStyle = theme.widget_text;
    ctx.font = "bold 11px sans-serif";
    ctx.textAlign = "end";
    const txt = Number.isFinite(ds.value) ? ds.value.toFixed(1) : "--";
    ctx.fillText(txt + (ds.units ? " " + ds.units : ""),
                 w - pad - 14, y + barH * 0.5 + 4);

    y += barH + gap;
  }

  ctx.textAlign = "start";
}
