// Status grid.
//
// Each dataset becomes a labelled card whose accent strip is coloured by
// the alarm thresholds: green when in range, amber when in the boundary
// zone, red when out of range, slate when the value isn't finite.

function status_color(ds) {
  const v = ds.value;
  if (!Number.isFinite(v)) return { tag: "no-data", color: theme.placeholder_text };

  const lo = ds.alarmLow;
  const hi = ds.alarmHigh;
  if (Number.isFinite(lo) && Number.isFinite(hi) && hi > lo) {
    if (v < lo || v > hi) return { tag: "alarm",   color: theme.alarm };
    const pad = (hi - lo) * 0.10;
    if (v < lo + pad || v > hi - pad) return { tag: "warn", color: theme.accent };
    return { tag: "ok", color: theme.widget_highlight };
  }
  return { tag: "info", color: theme.widget_highlight };
}

function paint(ctx, w, h) {
  // Cream paper background.
  ctx.fillStyle = theme.widget_base;
  ctx.fillRect(0, 0, w, h);

  const n = datasets.length;
  if (n === 0) return;

  // Header.
  const headerH = 22;
  ctx.fillStyle = theme.widget_text;
  ctx.font = "bold 11px sans-serif";
  ctx.textAlign = "start";
  ctx.fillText("STATUS  GRID", 14, 16);
  ctx.fillStyle = theme.placeholder_text;
  ctx.font = "10px sans-serif";
  ctx.textAlign = "end";
  ctx.fillText(n + " channels", w - 14, 16);
  ctx.fillStyle = theme.widget_border;
  ctx.fillRect(14, 20, w - 28, 1);

  const cols = Math.max(1, group.columns || Math.min(n, Math.max(1, Math.round(Math.sqrt(n * w / (h - headerH))))));
  const rows = Math.ceil(n / cols);
  const padX = 14;
  const padY = headerH + 6;
  const gap  = 8;
  const cellW = (w - padX * 2 - gap * (cols - 1)) / cols;
  const cellH = (h - padY - padX - gap * (rows - 1)) / rows;

  for (let i = 0; i < n; ++i) {
    const r = Math.floor(i / cols);
    const c = i % cols;
    const x = padX + c * (cellW + gap);
    const y = padY + r * (cellH + gap);
    const ds = datasets[i];
    const status = status_color(ds);

    // Card with shadow.
    ctx.fillStyle = theme.widget_border;
    ctx.fillRect(x + 1, y + 2, cellW, cellH);
    ctx.fillStyle = theme.alternate_base;
    ctx.fillRect(x, y, cellW, cellH);
    ctx.strokeStyle = theme.widget_border;
    ctx.lineWidth = 1;
    ctx.strokeRect(x + 0.5, y + 0.5, cellW - 1, cellH - 1);

    // Coloured accent strip on the left edge.
    ctx.fillStyle = status.color;
    ctx.fillRect(x, y, 4, cellH);

    // Status pill in the top-right corner.
    const pillW = 38;
    const pillH = 14;
    ctx.fillStyle = status.color;
    ctx.fillRect(x + cellW - pillW - 6, y + 6, pillW, pillH);
    ctx.fillStyle = theme.highlighted_text;
    ctx.font = "bold 9px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(status.tag.toUpperCase(), x + cellW - pillW * 0.5 - 6, y + 6 + pillH * 0.5);

    // Title (truncated).
    ctx.fillStyle = theme.placeholder_text;
    ctx.font = "10px sans-serif";
    ctx.textAlign = "start";
    ctx.textBaseline = "alphabetic";
    const title = (ds.title || "").substring(0, 28);
    ctx.fillText(title, x + 12, y + 24);

    // Value (large, bold). Measure width with the value's own font so the
    // units label lands flush to the right of the number.
    ctx.fillStyle = theme.widget_text;
    ctx.font = "bold 18px sans-serif";
    const value = Number.isFinite(ds.value)
      ? ds.value.toFixed(2)
      : "--";
    const valueW = ctx.measureTextWidth(value);
    ctx.fillText(value, x + 12, y + 50);

    if (ds.units) {
      ctx.fillStyle = theme.placeholder_text;
      ctx.font = "10px sans-serif";
      ctx.fillText(ds.units, x + 12 + valueW + 6, y + 50);
    }
  }

  ctx.textAlign = "start";
  ctx.textBaseline = "alphabetic";
}
