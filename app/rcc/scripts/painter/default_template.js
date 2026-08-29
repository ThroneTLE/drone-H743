// Default painter template.
//
// Renders datasets[0..2] as an X / Y / Z position indicator. The dot's planar
// position follows (X, Y); its size and colour follow Z. Edit this script
// freely -- it's a starting point.
//
// Available globals:
//   group              { id, title, columns, sourceId }
//   datasets           array-like; datasets[i] -> { value, rawValue, title, units, min, max, ... }
//   datasets.length    dataset count for this group
//   frame              { number, timestampMs }
//   datasetGetRaw(uid) cross-group dataset lookup (raw value)
//   datasetGetFinal(uid)  cross-group lookup (post-transform value)
//   tableGet/tableSet  Data Tables read/write
//
// You can also define `function onFrame() { ... }` to advance state once per
// parsed frame (history buffers, smoothing, etc.) before paint() runs.

function paint(ctx, w, h) {
  // Cream paper background + vignette.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  // Header strip.
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText((group.title || "PAINTER").toUpperCase(), 14, 18);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText("XY  /  Z", w - 14, 18);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 22, w - 28, 1);

  if (datasets.length < 2) {
    ctx.fillStyle = theme.placeholder_text;
    ctx.font = "12px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Add at least 2 datasets (X, Y) to see the indicator.",
                 w / 2, h / 2);
    return;
  }

  const dsX = datasets[0];
  const dsY = datasets[1];
  const dsZ = datasets.length >= 3 ? datasets[2] : null;

  // Plot card with shadow + border.
  const padL = 50;
  const padR = 132;
  const padT = 38;
  const padB = 38;
  const plotW = Math.max(1, w - padL - padR);
  const plotH = Math.max(1, h - padT - padB);
  const cx    = padL + plotW * 0.5;
  const cy    = padT + plotH * 0.5;

  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(padL + 1, padT + 2, plotW, plotH);
  ctx.fillStyle = theme.alternate_base;
  ctx.fillRect(padL, padT, plotW, plotH);
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.strokeRect(padL + 0.5, padT + 0.5, plotW - 1, plotH - 1);

  // Cross-hair grid.
  ctx.strokeStyle = theme.widget_border;
  ctx.beginPath();
  ctx.moveTo(padL, cy);
  ctx.lineTo(padL + plotW, cy);
  ctx.moveTo(cx, padT);
  ctx.lineTo(cx, padT + plotH);
  ctx.stroke();

  // Quarter divisions (subtle dotted feel via dashed slate ticks).
  ctx.strokeStyle = theme.widget_border;
  for (let i = 1; i < 4; ++i) {
    const x = padL + (plotW / 4) * i;
    const y = padT + (plotH / 4) * i;
    if (i !== 2) {
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, padT + plotH);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(padL + plotW, y);
      ctx.stroke();
    }
  }

  function norm(v, lo, hi) {
    if (!Number.isFinite(v) || hi <= lo) return 0.5;
    return Math.max(0, Math.min(1, (v - lo) / (hi - lo)));
  }

  const nx = norm(dsX.value, dsX.min, dsX.max);
  const ny = norm(dsY.value, dsY.min, dsY.max);
  const px = padL + nx * plotW;
  const py = padT + (1 - ny) * plotH;

  // Z drives both the dot size and the accent colour.
  let nz = 0.5;
  let zText = "--";
  if (dsZ && Number.isFinite(dsZ.value)) {
    nz = norm(dsZ.value, dsZ.min, dsZ.max);
    zText = dsZ.value.toFixed(2) + (dsZ.units ? " " + dsZ.units : "");
  }
  let dotColor = theme.widget_highlight;
  if (nz > 0.85) dotColor = theme.alarm;
  else if (nz > 0.66) dotColor = theme.accent;
  else if (nz < 0.20) dotColor = theme.widget_colors[5];
  const dotR = 6 + nz * 12;

  // Crosshairs from the dot to the axes.
  ctx.strokeStyle = theme.widget_border;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(padL, py);
  ctx.lineTo(px, py);
  ctx.moveTo(px, padT + plotH);
  ctx.lineTo(px, py);
  ctx.stroke();

  // Soft halo (lighter shade of the dot colour). Halos use the same
  // moveTo-before-arc discipline as the rest of the templates.
  ctx.fillStyle = theme.widget_border;
  ctx.beginPath();
  ctx.moveTo(px + dotR * 2.2, py);
  ctx.arc(px, py, dotR * 2.2, 0, Math.PI * 2);
  ctx.fill();

  // Filled dot.
  ctx.fillStyle = dotColor;
  ctx.beginPath();
  ctx.moveTo(px + dotR, py);
  ctx.arc(px, py, dotR, 0, Math.PI * 2);
  ctx.fill();

  // White outline + dark centre pip.
  ctx.strokeStyle = theme.widget_base;
  ctx.lineWidth   = 2;
  ctx.beginPath();
  ctx.moveTo(px + dotR, py);
  ctx.arc(px, py, dotR, 0, Math.PI * 2);
  ctx.stroke();

  ctx.fillStyle = theme.widget_text;
  ctx.beginPath();
  ctx.moveTo(px + 2, py);
  ctx.arc(px, py, 2, 0, Math.PI * 2);
  ctx.fill();

  // Axis labels.
  ctx.fillStyle    = theme.widget_text;
  ctx.font         = "10px sans-serif";
  ctx.textAlign    = "center";
  ctx.textBaseline = "top";
  ctx.fillText(dsX.title + (dsX.units ? " (" + dsX.units + ")" : ""),
               cx, padT + plotH + 8);

  ctx.save();
  ctx.translate(padL - 28, cy);
  ctx.rotate(-Math.PI * 0.5);
  ctx.textAlign    = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(dsY.title + (dsY.units ? " (" + dsY.units + ")" : ""), 0, 0);
  ctx.restore();
  ctx.textAlign    = "start";
  ctx.textBaseline = "alphabetic";

  // Side panel: per-channel readouts as small "stat" cards.
  const panelX = padL + plotW + 14;
  const panelY = padT;
  const cardW  = w - panelX - 14;
  const cardH  = 30;
  const gap    = 6;

  function stat_card(i, tag, value, color) {
    const y = panelY + i * (cardH + gap);
    // Card.
    ctx.fillStyle = theme.widget_border;
    ctx.fillRect(panelX + 1, y + 2, cardW, cardH);
    ctx.fillStyle = theme.alternate_base;
    ctx.fillRect(panelX, y, cardW, cardH);
    ctx.strokeStyle = theme.widget_border;
    ctx.lineWidth = 1;
    ctx.strokeRect(panelX + 0.5, y + 0.5, cardW - 1, cardH - 1);
    // Accent strip.
    ctx.fillStyle = color;
    ctx.fillRect(panelX, y, 3, cardH);
    // Tag.
    ctx.fillStyle = theme.placeholder_text;
    ctx.font = "bold 9px sans-serif";
    ctx.textAlign = "start";
    ctx.fillText(tag, panelX + 8, y + 12);
    // Value.
    ctx.fillStyle = theme.widget_text;
    ctx.font = "bold 13px sans-serif";
    ctx.fillText(value, panelX + 8, y + 25);
  }

  const xText = (Number.isFinite(dsX.value) ? dsX.value.toFixed(2) : "--") +
                (dsX.units ? " " + dsX.units : "");
  const yText = (Number.isFinite(dsY.value) ? dsY.value.toFixed(2) : "--") +
                (dsY.units ? " " + dsY.units : "");

  stat_card(0, "X", xText, theme.widget_colors[0]);
  stat_card(1, "Y", yText, theme.widget_colors[1]);
  if (dsZ) stat_card(2, "Z", zText, dotColor);
}
