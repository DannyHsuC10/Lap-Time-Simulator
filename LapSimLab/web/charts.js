const trackHue = 230;

export function bounds(values) {
  let min = Infinity;
  let max = -Infinity;
  for (const value of values) {
    min = Math.min(min, value);
    max = Math.max(max, value);
  }
  return [min, max];
}

function drawMarker(ctx, x, y) {
  ctx.beginPath();
  ctx.arc(x, y, 5, 0, 2 * Math.PI);
  ctx.fillStyle = "#f5f8fc";
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#111b25";
  ctx.stroke();
}

function drawTrack(ctx, width, height, result, values, sample) {
  const [xMin, xMax] = bounds(result.x);
  const [yMin, yMax] = bounds(result.y);
  const [min, max] = bounds(values);
  const scale = Math.min((width - 40) / (xMax - xMin || 1), (height - 40) / (yMax - yMin || 1));
  const x = (index) => width / 2 + (result.x[index] - (xMin + xMax) / 2) * scale;
  const y = (index) => height / 2 - (result.y[index] - (yMin + yMax) / 2) * scale;
  ctx.lineCap = "round";
  ctx.lineWidth = 4;
  for (let index = 1; index < values.length; index++) {
    const fraction = max === min ? 0 : ((values[index] + values[index - 1]) / 2 - min) / (max - min);
    ctx.strokeStyle = `hsl(${trackHue * (1 - fraction)} 86% 64%)`;
    ctx.beginPath();
    ctx.moveTo(x(index - 1), y(index - 1));
    ctx.lineTo(x(index), y(index));
    ctx.stroke();
  }
  drawMarker(ctx, x(sample), y(sample));
}

function tick(value) {
  return Number(value.toPrecision(4)).toString();
}

export const lineColors = ["#75e5ce", "#ffaf73", "#91b7ff", "#e5a1ff", "#f7df77"];

function drawSeries(ctx, width, height, series, xUnit, yUnit, sample) {
  const [low, high] = bounds(series.flatMap(({ values }) => values));
  const padding = (high - low || Math.abs(high) || 1) * 0.08;
  const min = low - padding;
  const max = high + padding;
  const [xStart, xEnd] = bounds(series.flatMap(({ x }) => x));
  const left = 57;
  const bottom = height - 47;
  const top = 29;
  const right = width - 22;
  const x = (value) => left + (value - xStart) / (xEnd - xStart || 1) * (right - left);
  const y = (value) => bottom - (value - min) / (max - min) * (bottom - top);
  ctx.font = "11px system-ui";
  ctx.fillStyle = "#a7b7c5";
  ctx.fillText(yUnit, 11, 17);
  ctx.textAlign = "right";
  ctx.fillText(xUnit, right, height - 8);
  ctx.strokeStyle = "#344554";
  ctx.lineWidth = 1;
  const columns = width < 420 ? 3 : 5;
  for (let index = 0; index <= columns; index++) {
    const position = left + (right - left) * index / columns;
    ctx.beginPath();
    ctx.moveTo(position, top);
    ctx.lineTo(position, bottom);
    ctx.stroke();
    ctx.textAlign = "center";
    ctx.fillText(tick(xStart + (xEnd - xStart) * index / columns), position, bottom + 18);
  }
  for (let index = 0; index <= 5; index++) {
    const value = min + (max - min) * index / 5;
    const position = y(value);
    ctx.beginPath();
    ctx.moveTo(left, position);
    ctx.lineTo(right, position);
    ctx.stroke();
    ctx.textAlign = "right";
    ctx.fillText(tick(value), left - 6, position + 4, left - 10);
  }
  for (const [line, { x: domain, values, color }] of series.entries()) {
    ctx.beginPath();
    ctx.moveTo(x(domain[0]), y(values[0]));
    for (let index = 1; index < values.length; index++) ctx.lineTo(x(domain[index]), y(values[index]));
    ctx.strokeStyle = color ?? lineColors[line % lineColors.length];
    ctx.lineWidth = 2;
    ctx.stroke();
  }
  if (sample !== undefined) {
    for (const { x: domain, values } of series) drawMarker(ctx, x(domain[sample]), y(values[sample]));
  }
}

export function paintCanvas(canvas, draw) {
  const { clientWidth: width, clientHeight: height } = canvas;
  if (!width || !height) return;
  const pixels = window.devicePixelRatio || 1;
  canvas.width = Math.round(width * pixels);
  canvas.height = Math.round(height * pixels);
  const ctx = canvas.getContext("2d");
  ctx.scale(pixels, pixels);
  draw(ctx, width, height);
}

export function drawLines(canvas, series, xUnit, yUnit, sample) {
  paintCanvas(canvas, (ctx, width, height) => drawSeries(ctx, width, height, series, xUnit, yUnit, sample));
}

export function drawChart(canvas, result, view, signal, sample) {
  if (view === "track") {
    paintCanvas(canvas, (ctx, width, height) => drawTrack(ctx, width, height, result, signal.values, sample));
  } else {
    drawLines(canvas, [{ x: result[view], values: signal.values }], view === "time" ? "s" : "m", signal.unit, sample);
  }
}
