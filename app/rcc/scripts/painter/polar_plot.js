// Polar plot.
//
// Treats datasets pairwise as (angle_deg, magnitude). Useful for radar-style
// or compass-rose visualisations: orientation, antenna patterns, gait phase,
// rotor diagnostics. Each pair becomes one ray with a coloured tip.

function paint(ctx, w, h) {
  // Cream paper background + vignette.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  const pairs = Math.floor(datasets.length / 2);

  // Header.
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("POLAR  PLOT", 14, 18);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(pairs + (pairs === 1 ? " ray" : " rays"), w - 14, 18);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 22, w - 28, 1);

  // Card containing the dial.
  const padX = 14;
  const padTop = 30;
  const padBot = 14;
  const cardX = padX;
  const cardY = padTop;
  const cardW = w - padX * 2;
  const cardH = h - padTop - padBot;

  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(cardX + 1, cardY + 2, cardW, cardH);
  ctx.fillStyle = theme.alternate_base;
  ctx.fillRect(cardX, cardY, cardW, cardH);
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.strokeRect(cardX + 0.5, cardY + 0.5, cardW - 1, cardH - 1);

  const cx = cardX + cardW * 0.5;
  const cy = cardY + cardH * 0.5;
  const r  = Math.max(20, Math.min(cardW, cardH) * 0.42);

  // Concentric range rings.
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth   = 1;
  for (let i = 1; i <= 4; ++i) {
    const rr = (r / 4) * i;
    ctx.beginPath();
    ctx.moveTo(cx + rr, cy);
    ctx.arc(cx, cy, rr, 0, Math.PI * 2);
    ctx.stroke();
  }

  // Cardinal cross.
  ctx.strokeStyle = theme.widget_border;
  ctx.beginPath();
  ctx.moveTo(cx - r, cy);
  ctx.lineTo(cx + r, cy);
  ctx.moveTo(cx, cy - r);
  ctx.lineTo(cx, cy + r);
  ctx.stroke();

  // Bearing spokes every 30 degrees.
  ctx.strokeStyle = theme.widget_border;
  for (let deg = 30; deg < 360; deg += 30) {
    if (deg % 90 === 0) continue;
    const a = (deg - 90) * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(a) * r, cy + Math.sin(a) * r);
    ctx.stroke();
  }

  // Cardinal labels.
  ctx.fillStyle    = theme.widget_text;
  ctx.font         = "bold 10px sans-serif";
  ctx.textAlign    = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("N", cx,           cy - r - 8);
  ctx.fillText("S", cx,           cy + r + 8);
  ctx.fillText("E", cx + r + 10,  cy);
  ctx.fillText("W", cx - r - 10,  cy);
  ctx.textBaseline = "alphabetic";

  // Outer ring.
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx + r, cy);
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.stroke();

  // Pairs.
  for (let i = 0; i + 1 < datasets.length; i += 2) {
    const angDeg = datasets[i].value;
    const mag    = datasets[i + 1].value;
    if (!Number.isFinite(angDeg) || !Number.isFinite(mag)) continue;

    const max  = datasets[i + 1].max || 1;
    const norm = Math.max(0, Math.min(1, mag / max));
    const ang  = (angDeg - 90) * Math.PI / 180;
    const x    = cx + Math.cos(ang) * r * norm;
    const y    = cy + Math.sin(ang) * r * norm;

    const color = theme.widget_colors[(i / 2 | 0) % theme.widget_colors.length];

    ctx.strokeStyle = color;
    ctx.lineWidth   = 2.5;
    ctx.lineCap     = "round";
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(x, y);
    ctx.stroke();

    // Tip with white halo.
    ctx.fillStyle = theme.widget_base;
    ctx.beginPath();
    ctx.moveTo(x + 5, y);
    ctx.arc(x, y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(x + 3, y);
    ctx.arc(x, y, 3, 0, Math.PI * 2);
    ctx.fill();
  }

  // Centre pivot.
  ctx.fillStyle = theme.widget_text;
  ctx.beginPath();
  ctx.moveTo(cx + 4, cy);
  ctx.arc(cx, cy, 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.lineCap = "butt";
  ctx.textAlign = "start";
}
