// XY scope (Lissajous mode).
//
// Treats datasets pairwise as (X, Y). For each pair, draws the latest
// trajectory in XY space (analog-oscilloscope X-Y mode). Useful for phase
// relationships, gait cycles, attractor visualisations.

const HISTORY = 200;
const trails  = [];

function onFrame() {
  const pairs = Math.floor(datasets.length / 2);
  while (trails.length < pairs) trails.push([]);

  for (let p = 0; p < pairs; ++p) {
    const x = datasets[p * 2].value;
    const y = datasets[p * 2 + 1].value;
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;

    trails[p].push([x, y]);
    if (trails[p].length > HISTORY) trails[p].shift();
  }
}

function paint(ctx, w, h) {
  // Cream paper background + vignette.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  // Header.
  const pairs = Math.max(trails.length, Math.floor(datasets.length / 2));
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("XY  SCOPE", 14, 18);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(pairs + (pairs === 1 ? " pair" : " pairs"), w - 14, 18);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 22, w - 28, 1);

  if (pairs === 0) {
    ctx.fillStyle = theme.placeholder_text;
    ctx.font = "12px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Add datasets in (X, Y) pairs", w / 2, h / 2);
    return;
  }

  // Layout: square-ish grid of cards.
  const cols = Math.ceil(Math.sqrt(pairs * w / h));
  const rows = Math.ceil(pairs / cols);
  const padX = 14;
  const padTop = 30;
  const padBot = 14;
  const gap  = 8;
  const cw   = (w - padX * 2 - gap * (cols - 1)) / cols;
  const ch   = (h - padTop - padBot - gap * (rows - 1)) / rows;

  for (let p = 0; p < pairs; ++p) {
    const r  = Math.floor(p / cols);
    const c  = p % cols;
    const x0 = padX + c * (cw + gap);
    const y0 = padTop + r * (ch + gap);
    const cx = x0 + cw * 0.5;
    const cy = y0 + ch * 0.5 + 6;
    const sz = Math.min(cw, ch - 16) * 0.42;

    // Card.
    ctx.fillStyle = theme.widget_border;
    ctx.fillRect(x0 + 1, y0 + 2, cw, ch);
    ctx.fillStyle = theme.alternate_base;
    ctx.fillRect(x0, y0, cw, ch);
    ctx.strokeStyle = theme.widget_border;
    ctx.lineWidth = 1;
    ctx.strokeRect(x0 + 0.5, y0 + 0.5, cw - 1, ch - 1);

    // Mini header.
    const dsX = datasets[p * 2];
    const dsY = datasets[p * 2 + 1];
    ctx.fillStyle = theme.placeholder_text;
    ctx.font = "bold 10px sans-serif";
    ctx.textAlign = "start";
    ctx.textBaseline = "alphabetic";
    const titleStr = (dsX.title || "X") + " x " + (dsY.title || "Y");
    ctx.fillText(titleStr.substring(0, 24), x0 + 8, y0 + 14);

    // Crosshair grid inside the card.
    ctx.strokeStyle = theme.widget_border;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx - sz, cy);    ctx.lineTo(cx + sz, cy);
    ctx.moveTo(cx, cy - sz);    ctx.lineTo(cx, cy + sz);
    ctx.stroke();

    // Outer plot frame.
    ctx.strokeStyle = theme.widget_border;
    ctx.strokeRect(cx - sz + 0.5, cy - sz + 0.5, sz * 2 - 1, sz * 2 - 1);

    const t   = trails[p];
    if (!t || t.length < 2) continue;
    const xLo = dsX.min, xHi = dsX.max;
    const yLo = dsY.min, yHi = dsY.max;
    const xSpan = (xHi - xLo) || 1;
    const ySpan = (yHi - yLo) || 1;
    const color = theme.widget_colors[p % theme.widget_colors.length];

    // Trail with fading alpha so motion direction is implicit.
    ctx.strokeStyle = color;
    ctx.lineWidth   = 1.6;
    for (let k = 1; k < t.length; ++k) {
      const a = t[k - 1], b = t[k];
      const xa = cx + ((a[0] - (xLo + xHi) * 0.5) / xSpan) * sz * 2;
      const ya = cy - ((a[1] - (yLo + yHi) * 0.5) / ySpan) * sz * 2;
      const xb = cx + ((b[0] - (xLo + xHi) * 0.5) / xSpan) * sz * 2;
      const yb = cy - ((b[1] - (yLo + yHi) * 0.5) / ySpan) * sz * 2;
      ctx.globalAlpha = 0.15 + 0.85 * (k / t.length);
      ctx.beginPath();
      ctx.moveTo(xa, ya);
      ctx.lineTo(xb, yb);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;

    // Latest position dot with halo.
    const last = t[t.length - 1];
    const lx = cx + ((last[0] - (xLo + xHi) * 0.5) / xSpan) * sz * 2;
    const ly = cy - ((last[1] - (yLo + yHi) * 0.5) / ySpan) * sz * 2;

    ctx.fillStyle = theme.widget_base;
    ctx.beginPath();
    ctx.moveTo(lx + 5, ly);
    ctx.arc(lx, ly, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(lx + 3, ly);
    ctx.arc(lx, ly, 3, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.textBaseline = "alphabetic";
  ctx.textAlign    = "start";
}
