// Round dial gauge with coloured zones, smoothed needle, and peak hold.
//
// Renders datasets[0] as an analog gauge with major / minor ticks, a tri-zone
// arc (green / amber / red), and a needle that smoothly tracks the value.
// A peak marker lingers above the needle and decays after a hold time.
//
// IMPORTANT: every partial arc is preceded by a moveTo at the arc's start
// point. PainterContext::arc maps to QPainterPath::arcTo, which connects
// the path's existing cursor to the arc start with a straight line --
// without the moveTo you get a chord across the dial.

const PEAK_HOLD_MS = 1200;
const SMOOTHING    = 0.18;

let smoothed = 0;
let peak     = 0;
let peakAt   = 0;
let lastTs   = 0;

function onFrame() {
  if (datasets.length === 0) return;
  const v = Number.isFinite(datasets[0].value) ? datasets[0].value : 0;

  const now = frame.timestampMs;
  const dt  = lastTs > 0 ? Math.max(0, (now - lastTs) * 1e-3) : 0;
  lastTs    = now;

  smoothed += SMOOTHING * (v - smoothed);

  const a = Math.abs(v);
  if (a > peak) { peak = a; peakAt = now; }
  else if (now - peakAt > PEAK_HOLD_MS) {
    const span = (datasets[0].max - datasets[0].min) || 1;
    peak = Math.max(0, peak - span * 0.04 * dt);
  }
}

function paint(ctx, w, h) {
  // Cream paper background -- looks great in screenshots, prints well.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  if (datasets.length === 0) return;

  const ds   = datasets[0];
  const v    = Number.isFinite(smoothed) ? smoothed : 0;
  const lo   = ds.min;
  const hi   = (ds.max > ds.min) ? ds.max : (ds.min + 100);
  const span = hi - lo;

  // Reserve a strip at the bottom for the value + units. Centre the dial
  // in the remaining square so the gauge sits cleanly at any aspect ratio.
  const labelH = 36;
  const margin = 12;
  const availW = w - margin * 2;
  const availH = h - margin * 2 - labelH;
  const r      = Math.max(8, Math.min(availW, availH) * 0.5 - 8);
  const cx     = w / 2;
  const cy     = margin + r + 8;

  const startA = Math.PI * 0.85;
  const endA   = Math.PI * 2.15;
  const sweep  = endA - startA;

  // Coloured zones along the arc.
  function zone(v0, v1, color) {
    const t0 = startA + sweep * ((v0 - lo) / span);
    const t1 = startA + sweep * ((v1 - lo) / span);
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(t0) * r, cy + Math.sin(t0) * r);
    ctx.arc(cx, cy, r, t0, t1);
    ctx.lineWidth   = 14;
    ctx.lineCap     = "butt";
    ctx.strokeStyle = color;
    ctx.stroke();
  }
  zone(lo,                lo + span * 0.60, theme.widget_highlight);
  zone(lo + span * 0.60, lo + span * 0.85, theme.accent);
  zone(lo + span * 0.85, hi,                theme.alarm);

  // Tick marks (major every 10%, minor every 5%).
  ctx.strokeStyle = theme.widget_text;
  for (let i = 0; i <= 20; ++i) {
    const t      = startA + sweep * (i / 20);
    const major  = (i % 2 === 0);
    const inner  = r - (major ? 22 : 14);
    const outer  = r - 6;
    ctx.lineWidth = major ? 2 : 1;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(t) * inner, cy + Math.sin(t) * inner);
    ctx.lineTo(cx + Math.cos(t) * outer, cy + Math.sin(t) * outer);
    ctx.stroke();
  }

  // Major tick labels.
  ctx.fillStyle    = theme.widget_text;
  ctx.font         = "bold 10px sans-serif";
  ctx.textAlign    = "center";
  ctx.textBaseline = "middle";
  for (let i = 0; i <= 5; ++i) {
    const t  = startA + sweep * (i / 5);
    const tx = cx + Math.cos(t) * (r - 34);
    const ty = cy + Math.sin(t) * (r - 34);
    ctx.fillText((lo + (span * i / 5)).toFixed(0), tx, ty);
  }

  // Peak marker -- a small amber tick on the arc.
  if (peak > 0) {
    const pn = Math.max(0, Math.min(1, (peak - lo) / span));
    const tp = startA + sweep * pn;
    ctx.strokeStyle = theme.accent;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(tp) * (r - 22), cy + Math.sin(tp) * (r - 22));
    ctx.lineTo(cx + Math.cos(tp) * (r + 4),  cy + Math.sin(tp) * (r + 4));
    ctx.stroke();
  }

  // Needle.
  const norm = Math.max(0, Math.min(1, (v - lo) / span));
  const valA = startA + sweep * norm;
  ctx.strokeStyle = theme.widget_text;
  ctx.lineWidth = 3;
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + Math.cos(valA) * (r - 12), cy + Math.sin(valA) * (r - 12));
  ctx.stroke();
  ctx.lineCap = "butt";

  // Pivot cap.
  ctx.fillStyle = theme.widget_text;
  ctx.beginPath();
  ctx.moveTo(cx + 6, cy);
  ctx.arc(cx, cy, 6, 0, Math.PI * 2);
  ctx.fill();

  // Value & label below the dial.
  const labelY = cy + r + 22;
  ctx.fillStyle    = theme.placeholder_text;
  ctx.font         = "10px sans-serif";
  ctx.textAlign    = "center";
  ctx.textBaseline = "alphabetic";
  ctx.fillText(ds.title + (ds.units ? "  (" + ds.units + ")" : ""), cx, labelY);

  ctx.fillStyle = theme.widget_text;
  ctx.font      = "bold 18px sans-serif";
  ctx.fillText(v.toFixed(2) + (ds.units ? " " + ds.units : ""), cx, labelY + 20);

  ctx.textAlign = "start";
}
