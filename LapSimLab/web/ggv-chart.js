import { paintCanvas } from "./charts.js";

export const defaultAngle = { azimuth: -0.64, elevation: 0.46 };

export function drawGgvSurface(canvas, slices, angle) {
  paintCanvas(canvas, (ctx, width, height) => {
    const maxAy = Math.max(0.1, ...slices.flatMap(({ ay }) => ay.map(Math.abs)));
    const maxAx = Math.max(0.1, ...slices.flatMap(({ drive, brake }) => [...drive, ...brake].map(Math.abs)));
    const maxSpeed = slices.at(-1).speed;
    const ca = Math.cos(angle.azimuth), sa = Math.sin(angle.azimuth);
    const ce = Math.cos(angle.elevation), se = Math.sin(angle.elevation);
    const scale = Math.min(width * 0.29, height * 0.30);
    function project(ay, ax, speed) {
      const x = ay / maxAy;
      const y = ax / maxAx;
      const z = 1.7 * (speed / maxSpeed - 0.5);
      const depth = x * sa + y * ca;
      return [width / 2 + (x * ca - y * sa) * scale,
        height / 2 - (z * ce - depth * se) * scale, depth * ce + z * se];
    }

    const faces = [];
    for (const branch of ["drive", "brake"]) {
      const bands = slices.map(({ speed, ay, [branch]: ax }) => ay.map((value, index) => project(value, ax[index], speed)));
      for (let row = 1; row < bands.length; row++) {
        const previous = bands[row - 1];
        const current = bands[row];
        for (let column = 1; column < current.length; column++) {
          const corners = [previous[column - 1], previous[column], current[column], current[column - 1]];
          const depth = corners.reduce((sum, point) => sum + point[2], 0) / corners.length;
          const lightness = 34 + 25 * row / (bands.length - 1);
          faces.push({ corners, depth, fill: `hsl(${branch === "drive" ? 174 : 27} 60% ${lightness}%)` });
        }
      }
    }
    faces.sort((a, b) => a.depth - b.depth);
    ctx.strokeStyle = "#111d3160";
    ctx.lineWidth = 0.65;
    for (const { corners, fill } of faces) {
      ctx.beginPath();
      ctx.moveTo(...corners[0].slice(0, 2));
      for (const [x, y] of corners.slice(1)) ctx.lineTo(x, y);
      ctx.closePath();
      ctx.fillStyle = fill;
      ctx.fill();
      ctx.stroke();
    }

    const origin = project(-maxAy, -maxAx, 0);
    const axes = [
      [project(maxAy, -maxAx, 0), "ay (g)"],
      [project(-maxAy, maxAx, 0), "ax (g)"],
      [project(-maxAy, -maxAx, maxSpeed), `${Number(maxSpeed.toFixed(1))} km/h`],
    ];
    ctx.font = '12px "Segoe UI", sans-serif';
    ctx.strokeStyle = "#475569";
    ctx.fillStyle = "#142638";
    ctx.lineWidth = 1.25;
    for (const [[x, y], label] of axes) {
      ctx.beginPath();
      ctx.moveTo(origin[0], origin[1]);
      ctx.lineTo(x, y);
      ctx.stroke();
      const labelX = Math.max(5, Math.min(width - ctx.measureText(label).width - 5, x + 5));
      ctx.fillText(label, labelX, Math.max(14, Math.min(height - 5, y - 5)));
    }
  });
}
