// 7-segment LCD display.
//
// Renders datasets[0..N-1] as classic 7-segment digits stacked in rows --
// the same look as a calculator or a desktop multimeter. Each row sits
// inside a recessed "screen" panel with a soft mint background and dark
// segments, with faint "ghost" segments showing the unlit cells.

const SEG = {
  // Segments per digit: a, b, c, d, e, f, g
  "0": [1, 1, 1, 1, 1, 1, 0],
  "1": [0, 1, 1, 0, 0, 0, 0],
  "2": [1, 1, 0, 1, 1, 0, 1],
  "3": [1, 1, 1, 1, 0, 0, 1],
  "4": [0, 1, 1, 0, 0, 1, 1],
  "5": [1, 0, 1, 1, 0, 1, 1],
  "6": [1, 0, 1, 1, 1, 1, 1],
  "7": [1, 1, 1, 0, 0, 0, 0],
  "8": [1, 1, 1, 1, 1, 1, 1],
  "9": [1, 1, 1, 1, 0, 1, 1],
  "-": [0, 0, 0, 0, 0, 0, 1],
  " ": [0, 0, 0, 0, 0, 0, 0]
};

function drawDigit(ctx, x, y, w, h, ch) {
  const segs = SEG[ch] || SEG[" "];
  const t    = Math.max(2, h * 0.10);

  function bar(px, py, pw, ph, on) {
    ctx.fillStyle = on ? theme.widget_text : theme.widget_border;
    ctx.fillRect(px, py, pw, ph);
  }

  bar(x + t, y,                       w - 2 * t, t, segs[0]);                   // a
  bar(x + w - t, y + t,               t,         h * 0.5 - t * 1.5, segs[1]);   // b
  bar(x + w - t, y + h * 0.5 + t * 0.5, t,       h * 0.5 - t * 1.5, segs[2]);   // c
  bar(x + t, y + h - t,               w - 2 * t, t, segs[3]);                   // d
  bar(x,     y + h * 0.5 + t * 0.5,   t,         h * 0.5 - t * 1.5, segs[4]);   // e
  bar(x,     y + t,                   t,         h * 0.5 - t * 1.5, segs[5]);   // f
  bar(x + t, y + h * 0.5 - t * 0.5,   w - 2 * t, t, segs[6]);                   // g
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
  ctx.fillText("7-SEGMENT  READOUT", 14, 18);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(datasets.length + (datasets.length === 1 ? " channel" : " channels"),
               w - 14, 18);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 22, w - 28, 1);

  // Per-row stacked layout.
  const padX = 14;
  const padTop = 32;
  const padBot = 14;
  const gap  = 8;
  const rowH = (h - padTop - padBot - gap * (datasets.length - 1)) / datasets.length;

  for (let i = 0; i < datasets.length; ++i) {
    const ds  = datasets[i];
    const y0  = padTop + i * (rowH + gap);
    const v   = Number.isFinite(ds.value) ? ds.value : 0;
    const txt = v.toFixed(1).padStart(6, " ");

    // "Screen" panel (mint background simulating a backlit LCD).
    ctx.fillStyle = theme.widget_border;
    ctx.fillRect(padX + 1, y0 + 2, w - padX * 2, rowH);
    ctx.fillStyle = theme.alternate_base;
    ctx.fillRect(padX, y0, w - padX * 2, rowH);
    ctx.strokeStyle = theme.widget_border;
    ctx.lineWidth = 1;
    ctx.strokeRect(padX + 0.5, y0 + 0.5, w - padX * 2 - 1, rowH - 1);

    // Title strip.
    ctx.fillStyle = theme.widget_text;
    ctx.font = "bold 9px sans-serif";
    ctx.textAlign = "start";
    ctx.textBaseline = "alphabetic";
    ctx.fillText((ds.title || ("CH " + (i + 1))).toUpperCase(),
                 padX + 8, y0 + 12);
    if (ds.units) {
      ctx.fillStyle = theme.placeholder_text;
      ctx.font = "9px sans-serif";
      ctx.textAlign = "end";
      ctx.fillText(ds.units, w - padX - 8, y0 + 12);
    }

    // Digits.
    const dh = Math.max(12, rowH - 22);
    const dw = dh * 0.55;
    const dgap = dw * 0.20;
    const totalW = txt.length * (dw + dgap);
    const xT = padX + (w - padX * 2 - totalW) * 0.5;
    const yT = y0 + 16;

    let cx = xT;
    for (let k = 0; k < txt.length; ++k) {
      const ch = txt[k];
      if (ch === ".") {
        // Decimal point as a small square at baseline.
        ctx.fillStyle = theme.widget_text;
        ctx.fillRect(cx - dw * 0.10, yT + dh - dw * 0.18,
                     dw * 0.18, dw * 0.18);
        continue;
      }
      drawDigit(ctx, cx, yT, dw, dh, ch);
      cx += dw + dgap;
    }
  }

  ctx.textBaseline = "alphabetic";
  ctx.textAlign    = "start";
}
