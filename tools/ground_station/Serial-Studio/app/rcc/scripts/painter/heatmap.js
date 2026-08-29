// Heatmap.
//
// Treats datasets[0..N-1] as a row vector pushed onto a scrolling 2D image.
// Shows a colour-mapped time-vs-channel heatmap, useful for spectrograms,
// thermal arrays, or multi-channel snapshots over time. The viridis
// colour-map keeps cells perceptually uniform.

const HISTORY = 120;
const rows = [];
let lastLo = 0;
let lastHi = 1;

function onFrame() {
  const n = datasets.length;
  if (n === 0) return;

  let lo = Number.POSITIVE_INFINITY;
  let hi = Number.NEGATIVE_INFINITY;
  for (let i = 0; i < n; ++i) {
    lo = Math.min(lo, datasets[i].min);
    hi = Math.max(hi, datasets[i].max);
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi) || hi <= lo) {
    lo = 0;
    hi = 1;
  }
  lastLo = lo;
  lastHi = hi;

  const row = new Array(n);
  for (let i = 0; i < n; ++i) {
    const v = datasets[i].value;
    row[i] = Number.isFinite(v) ? Math.max(0, Math.min(1, (v - lo) / (hi - lo))) : 0;
  }

  rows.push(row);
  if (rows.length > HISTORY) rows.shift();
}

function viridis(t) {
  const stops = [
    [0.000,  68,   1,  84],
    [0.250,  59,  82, 139],
    [0.500,  33, 145, 140],
    [0.750,  94, 201,  98],
    [1.000, 253, 231,  37]
  ];
  for (let i = 1; i < stops.length; ++i) {
    if (t <= stops[i][0]) {
      const a = stops[i - 1], b = stops[i];
      const k = (t - a[0]) / (b[0] - a[0]);
      const r = a[1] + (b[1] - a[1]) * k;
      const g = a[2] + (b[2] - a[2]) * k;
      const bl = a[3] + (b[3] - a[3]) * k;
      return `rgb(${r | 0}, ${g | 0}, ${bl | 0})`;
    }
  }
  return "rgb(253, 231, 37)";
}

function paint(ctx, w, h) {
  // Cream paper background + vignette.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  // Header.
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("HEATMAP", 14, 18);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(datasets.length + " channels  /  viridis", w - 14, 18);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 22, w - 28, 1);

  if (rows.length === 0 || datasets.length === 0) return;

  // Plot card frames the heatmap. Card padding leaves room for the
  // colour scale on the right.
  const padX = 14;
  const padTop = 30;
  const padBot = 30;
  const scaleW = 22;
  const scaleGap = 8;
  const cardX = padX;
  const cardY = padTop;
  const cardW = w - padX * 2;
  const cardH = h - padTop - padBot;

  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(cardX + 1, cardY + 2, cardW, cardH);
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(cardX, cardY, cardW, cardH);

  const plotX = cardX + 1;
  const plotY = cardY + 1;
  const plotW = cardW - scaleW - scaleGap - 2;
  const plotH = cardH - 2;

  const cols = datasets.length;
  const cellW = plotW / HISTORY;
  const cellH = plotH / cols;

  for (let r = 0; r < rows.length; ++r) {
    const row = rows[r];
    const x   = plotX + r * cellW;
    for (let c = 0; c < cols; ++c) {
      ctx.fillStyle = viridis(row[c]);
      ctx.fillRect(x, plotY + c * cellH, cellW + 1, cellH + 1);
    }
  }

  // Card border on top.
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.strokeRect(cardX + 0.5, cardY + 0.5, cardW - 1, cardH - 1);

  // Vertical separator before the colour scale.
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(plotX + plotW, plotY, 1, plotH);

  // Colour scale (10 segments, value labels at top + bottom).
  const scaleX = plotX + plotW + scaleGap;
  const SEG = 32;
  const segH = plotH / SEG;
  for (let i = 0; i < SEG; ++i) {
    const t = 1 - (i + 0.5) / SEG;
    ctx.fillStyle = viridis(t);
    ctx.fillRect(scaleX, plotY + i * segH, scaleW, segH + 1);
  }
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.strokeRect(scaleX + 0.5, plotY + 0.5, scaleW - 1, plotH - 1);

  // Scale labels.
  ctx.fillStyle    = theme.widget_text;
  ctx.font         = "9px sans-serif";
  ctx.textAlign    = "start";
  ctx.textBaseline = "alphabetic";
  ctx.fillText(lastHi.toFixed(2), scaleX + scaleW + 4, plotY + 8);
  ctx.fillText(lastLo.toFixed(2), scaleX + scaleW + 4, plotY + plotH - 2);

  // Time axis caption (newest right, oldest left).
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "9px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("oldest", plotX, h - padBot + 14);
  ctx.textAlign = "end";
  ctx.fillText("newest", plotX + plotW, h - padBot + 14);

  ctx.textAlign = "start";
}
