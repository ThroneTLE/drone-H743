// Oscilloscope.
//
// Per-channel trace with a phosphor-green CRT background. Channels are
// spread vertically so traces don't overlap. Kept dark on purpose -- the
// "back-lit instrument" look is what makes a scope recognisable as a
// scope. The cream-paper card frame around the screen ties it back to the
// rest of the template family.

const HISTORY = 256;
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

function paint(ctx, w, h) {
  // Cream paper background framing the dark instrument screen.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  // Header strip.
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("OSCILLOSCOPE", 14, 18);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(datasets.length + (datasets.length === 1 ? " channel" : " channels"),
               w - 14, 18);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 22, w - 28, 1);

  // CRT screen (a dark recessed panel inside the card area).
  const padX = 14;
  const padTop = 30;
  const padBot = 14;
  const sx = padX;
  const sy = padTop;
  const sw = w - padX * 2;
  const sh = h - padTop - padBot;

  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(sx + 1, sy + 2, sw, sh);
  ctx.fillStyle = theme.alternate_base;
  ctx.fillRect(sx, sy, sw, sh);

  // Major + minor grid (10 vertical x 8 horizontal divisions).
  for (let i = 1; i < 10; ++i) {
    const x = sx + (sw / 10) * i;
    ctx.strokeStyle = theme.widget_border;
    ctx.lineWidth   = (i === 5) ? 1.5 : 1;
    ctx.beginPath();
    ctx.moveTo(x, sy);
    ctx.lineTo(x, sy + sh);
    ctx.stroke();
  }
  for (let i = 1; i < 8; ++i) {
    const y = sy + (sh / 8) * i;
    ctx.strokeStyle = theme.widget_border;
    ctx.lineWidth   = (i === 4) ? 1.5 : 1;
    ctx.beginPath();
    ctx.moveTo(sx, y);
    ctx.lineTo(sx + sw, y);
    ctx.stroke();
  }

  // Outer phosphor bezel.
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth   = 1;
  ctx.strokeRect(sx + 0.5, sy + 0.5, sw - 1, sh - 1);

  if (datasets.length === 0) return;

  const lanes = datasets.length;
  const laneH = sh / lanes;

  for (let i = 0; i < traces.length; ++i) {
    const t  = traces[i];
    if (t.length < 2) continue;
    const ds = datasets[i];
    const lo = ds.min;
    const hi = ds.max;
    const span = (hi - lo) || 1;
    const cy = sy + laneH * (i + 0.5);
    const traceColor = theme.widget_colors[i % theme.widget_colors.length];

    function trace_path() {
      ctx.beginPath();
      for (let k = 0; k < t.length; ++k) {
        const x = sx + (k / (HISTORY - 1)) * sw;
        const n = (t[k] - lo) / span;
        const y = cy - (n - 0.5) * laneH * 0.85;
        if (k === 0) ctx.moveTo(x, y);
        else         ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    // Glow underlay (wider, lighter shade).
    ctx.strokeStyle = traceColor;
    ctx.lineWidth   = 4;
    ctx.globalAlpha = 0.18;
    trace_path();
    ctx.globalAlpha = 1;

    // Crisp trace on top.
    ctx.strokeStyle = traceColor;
    ctx.lineWidth   = 2;
    trace_path();

    // Channel tag.
    ctx.fillStyle    = traceColor;
    ctx.font         = "bold 10px monospace";
    ctx.textAlign    = "start";
    ctx.textBaseline = "top";
    ctx.fillText("CH" + (i + 1) + "  " + (ds.title || ""),
                 sx + 8, sy + i * laneH + 6);
  }

  ctx.textBaseline = "alphabetic";
  ctx.textAlign    = "start";
}
