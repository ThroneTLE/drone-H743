// LED matrix.
//
// Each dataset becomes a row of LEDs lit proportionally to its value.
// Saturated colours mark active LEDs (green / amber / crimson by zone),
// pastel colours mark inactive ones so the bar's full length is always
// visible.

const LEDS_PER_ROW = 16;

function lit_color(t) {
  if (t < 0.66) return theme.widget_highlight;
  if (t < 0.85) return theme.accent;
  return theme.alarm;
}

function unlit_color(t) {
  return theme.widget_border;
}

function paint(ctx, w, h) {
  // Cream paper background + vignette.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  if (datasets.length === 0) return;

  // Header.
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("LED  MATRIX", 14, 18);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(LEDS_PER_ROW + " LEDs / row", w - 14, 18);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 22, w - 28, 1);

  // Card body.
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

  // Per-row layout.
  const labelW   = 78;
  const valueW   = 60;
  const innerPad = 12;
  const rowH     = cardH / datasets.length;
  const ledH     = Math.max(8, Math.min(rowH * 0.55, 22));
  const ledArea  = cardW - innerPad * 2 - labelW - valueW - 16;
  const ledGap   = 2;
  const ledW     = (ledArea - ledGap * (LEDS_PER_ROW - 1)) / LEDS_PER_ROW;

  for (let i = 0; i < datasets.length; ++i) {
    const ds   = datasets[i];
    const span = (ds.max - ds.min) || 1;
    const v    = Number.isFinite(ds.value) ? ds.value : ds.min;
    const norm = Math.max(0, Math.min(1, (v - ds.min) / span));
    const lit  = Math.round(norm * LEDS_PER_ROW);
    const yMid = cardY + i * rowH + rowH * 0.5;

    // Channel label on the left.
    ctx.fillStyle    = theme.widget_text;
    ctx.font         = "bold 11px sans-serif";
    ctx.textAlign    = "start";
    ctx.textBaseline = "middle";
    ctx.fillText((ds.title || ("CH " + (i + 1))).substring(0, 14),
                 cardX + innerPad, yMid);

    // LEDs.
    const ledX0 = cardX + innerPad + labelW + 8;
    const ledY  = yMid - ledH * 0.5;
    for (let k = 0; k < LEDS_PER_ROW; ++k) {
      const t  = (k + 0.5) / LEDS_PER_ROW;
      const on = (k < lit);
      ctx.fillStyle = on ? lit_color(t) : unlit_color(t);
      ctx.fillRect(ledX0 + k * (ledW + ledGap), ledY, ledW, ledH);
    }

    // Numeric readout on the right.
    ctx.fillStyle = theme.widget_text;
    ctx.font      = "bold 11px sans-serif";
    ctx.textAlign = "end";
    const txt = (Number.isFinite(ds.value) ? ds.value.toFixed(1) : "--") +
                (ds.units ? " " + ds.units : "");
    ctx.fillText(txt, cardX + cardW - innerPad, yMid);

    // Faint row separator.
    if (i < datasets.length - 1) {
      ctx.fillStyle = theme.widget_border;
      ctx.fillRect(cardX + innerPad, cardY + (i + 1) * rowH - 0.5,
                   cardW - innerPad * 2, 1);
    }
  }

  ctx.textBaseline = "alphabetic";
  ctx.textAlign    = "start";
}
