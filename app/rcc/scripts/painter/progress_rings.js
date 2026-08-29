// Progress rings.
//
// One circular ring per dataset, filled proportionally to its value. Rings
// arrange into a grid that fits the widget bounds at any aspect ratio.
//
// IMPORTANT: each partial value-arc is preceded by a moveTo at the arc's
// start point so PainterContext::arc (which maps to QPainterPath::arcTo)
// doesn't draw a stray chord from the implicit origin to the arc start.

function paint(ctx, w, h) {
  ctx.fillStyle = theme.groupbox_background;
  ctx.fillRect(0, 0, w, h);

  if (datasets.length === 0) return;

  // Lay rings out on a grid that prefers near-square cells.
  const n    = datasets.length;
  const cols = Math.min(n, Math.max(1, Math.round(Math.sqrt(n * w / h))));
  const rows = Math.ceil(n / cols);
  const pad  = 12;
  const cw   = (w - pad * 2) / cols;
  const ch   = (h - pad * 2) / rows;

  for (let i = 0; i < n; ++i) {
    const row  = Math.floor(i / cols);
    const col  = i % cols;
    const cx   = pad + cw * col + cw * 0.5;
    const cy   = pad + ch * row + ch * 0.5 - 6;
    const rr   = Math.max(10, Math.min(cw, ch) * 0.38 - 4);
    const ds   = datasets[i];
    const v    = Number.isFinite(ds.value) ? ds.value : 0;
    const span = (ds.max - ds.min) || 1;
    const norm = Math.max(0, Math.min(1, (v - ds.min) / span));

    // Track ring (full circle, stroke).
    ctx.strokeStyle = theme.mid;
    ctx.lineWidth   = 12;
    ctx.lineCap     = "butt";
    ctx.beginPath();
    ctx.moveTo(cx + rr, cy);
    ctx.arc(cx, cy, rr, 0, Math.PI * 2);
    ctx.stroke();

    // Value arc.
    if (norm > 0) {
      const startA = -Math.PI * 0.5;
      const endA   = startA + Math.PI * 2 * norm;
      ctx.strokeStyle = theme.highlight;
      ctx.lineCap     = "round";
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(startA) * rr, cy + Math.sin(startA) * rr);
      ctx.arc(cx, cy, rr, startA, endA);
      ctx.stroke();
      ctx.lineCap = "butt";
    }

    // Centre value (% of range) and small title underneath.
    ctx.fillStyle    = theme.text;
    ctx.font         = "bold 18px sans-serif";
    ctx.textAlign    = "center";
    ctx.textBaseline = "middle";
    ctx.fillText((norm * 100).toFixed(0) + "%", cx, cy);

    ctx.fillStyle    = theme.pane_section_label || theme.text;
    ctx.font         = "10px sans-serif";
    ctx.textBaseline = "alphabetic";
    const title = (ds.title || "").substring(0, 16);
    ctx.fillText(title, cx, cy + rr + 18);
  }

  ctx.textAlign    = "start";
  ctx.textBaseline = "alphabetic";
}
