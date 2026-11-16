import { bounds, drawLines, lineColors } from "./charts.js";
import { defaultAngle, drawGgvSurface } from "./ggv-chart.js";

const titles = { motor: "馬達曲線", aero: "空力掃描", tire: "輪胎力掃描" };
const aeroViews = {
  forces: ["N", [["下壓力", "F_down"], ["阻力", "F_drag"]]],
  axles: ["N", [["前軸", "front"], ["後軸", "rear"]]],
  coefficients: ["無因次", [["CL", "CL"], ["CD", "CD"]]],
};

export function installCurveTools(request, cancel) {
  const byId = (id) => document.getElementById(id);
  const dialog = byId("curve-dialog");
  const ggvPanel = byId("ggv-panel");
  const title = byId("curves-title");
  const form = byId("curve-form");
  const canvas = byId("curve-chart");
  const ggvCanvas = byId("ggv-chart");
  const sample = byId("curve-sample");
  const tabs = [...document.querySelectorAll(".tool-tabs [data-tool]")];
  const results = new Map();
  const changed = new Set();
  const angle = { ...defaultAngle };
  let current;
  let opener;
  let busy = true;
  let drag;

  function status(tool, message, error = false) {
    const note = byId(tool === "ggv" ? "ggv-status" : "curve-status");
    note.textContent = message;
    note.classList.toggle("error", error);
  }

  function legend(id, entries) {
    byId(id).replaceChildren(...entries.map(({ label, color }, index) => {
      const item = document.createElement("span");
      const swatch = document.createElement("i");
      swatch.style.backgroundColor = color ?? lineColors[index % lineColors.length];
      item.append(swatch, label);
      return item;
    }));
  }

  function drawInspection() {
    if (!dialog.open) return;
    const entry = results.get(current);
    byId("curve-empty").hidden = Boolean(entry);
    byId("curve-output").hidden = !entry;
    if (!entry) return;
    const { result, fileName } = entry;
    let message;
    let xUnit;
    let yUnit;
    let lines;
    if (current === "motor") {
      xUnit = "馬達轉速（RPM）";
      yUnit = "扭矩（N·m）";
      lines = [{ x: result.rpm, values: result.torque, label: "單馬達扭矩" }];
      message = `${fileName} · ${result.rpm.length} 筆 · 峰值 ${bounds(result.torque)[1].toFixed(2)} N·m`;
      if (result.voltage !== null) message += ` · ${result.voltage} V`;
    } else if (current === "aero") {
      const [unit, series] = aeroViews[byId("aero-metric").value];
      xUnit = `${result.axis}（°）`;
      yUnit = unit;
      lines = series.map(([label, key]) => ({ label, x: result.angles, values: result.signals[key] }));
      message = `${result.axis === "Roll" ? "Pitch" : "Roll"} 固定 ${result.fixed}°`;
      if (!result.enabled) message += " · 空力已關閉";
    } else {
      const metric = byId("tire-metric");
      const wheel = byId("tire-wheel");
      for (const option of Array.from(wheel.options).slice(1)) option.disabled = result.pointMass;
      if (result.pointMass) wheel.value = "all";
      const values = result[metric.value].map((row) => wheel.value === "all" ?
        row.reduce((sum, force) => sum + force, 0) : row[Number(wheel.value)]);
      xUnit = "Roll（°）";
      yUnit = "力（N）";
      lines = [{ label: metric.selectedOptions[0].textContent, x: result.angles, values }];
      message = result.pointMass ? "質點模型 · 整車抓地力" : "固定靜態輪載，Roll 用於換算外傾角";
      if (metric.value !== "peaks") {
        message += ` · ay = ${result.ay} g`;
        if (result.failures) message += ` · ${result.failures} 點無法滿足側向力需求，Fx 顯示為零`;
      }
    }
    sample.max = lines[0].x.length - 1;
    const marker = sample.valueAsNumber;
    canvas.setAttribute("aria-label", `${titles[current]}，${xUnit}、${yUnit}`);
    drawLines(canvas, lines, xUnit, yUnit, marker);
    legend("curve-legend", lines);
    byId("curve-readout").textContent = [
      `${xUnit} ${lines[0].x[marker].toFixed(2)}`,
      ...lines.map(({ label, values }) => `${label} ${values[marker].toFixed(2)} ${yUnit}`),
    ].join(" · ");
    if (!busy) status(current, changed.has(current) ? `設定已變更，請更新圖表。${message}` : message);
  }

  function drawGgv() {
    if (ggvPanel.hidden) return;
    const entry = results.get("ggv");
    const surface = byId("ggv-view").value === "surface";
    byId("ggv-empty").hidden = Boolean(entry);
    byId("ggv-output").hidden = !entry;
    byId("ggv-speed-label").hidden = surface || !entry;
    if (!entry) return;
    const { result } = entry;
    byId("ggv-rotation").hidden = !surface;
    ggvCanvas.classList.toggle("surface", surface);
    let message;
    if (surface) {
      ggvCanvas.setAttribute("aria-label", "GGV 3D 曲面，橫向加速度 ay、縱向加速度 ax 與車速");
      drawGgvSurface(ggvCanvas, result, angle);
      legend("ggv-legend", [{ label: "驅動", color: "#50c6a9" }, { label: "煞車", color: "#e6a064" }]);
      message = `${result.length} 個車速切片。拖曳旋轉，雙擊重設視角。`;
    } else {
      const speed = byId("ggv-speed").value;
      const slices = speed === "all" ? result.filter((_, index) => index % 3 === 0) : [result[Number(speed)]];
      const colors = slices.map(({ speed: kmh }, index) => ({ label: `${Number(kmh.toFixed(2))} km/h`, color: lineColors[index] }));
      const lines = slices.flatMap(({ ay, drive, brake }, index) => [
        { x: ay, values: drive, ...colors[index] }, { x: ay, values: brake, ...colors[index] },
      ]);
      ggvCanvas.setAttribute("aria-label", "GGV 2D 切片，橫向加速度 ay 與縱向加速度 ax");
      drawLines(ggvCanvas, lines, "橫向加速度 ay（g）", "縱向加速度 ax（g）");
      legend("ggv-legend", colors);
      message = "正 ax 為加速，負 ax 為煞車。ay 正值為左彎。";
      if (speed !== "all") {
        const { ay, drive, brake } = slices[0];
        message = `左彎 ${ay.at(-1).toFixed(2)} g · 右彎 ${Math.abs(ay[0]).toFixed(2)} g · ` +
          `加速 ${Math.max(...drive).toFixed(2)} g · 煞車 ${Math.abs(Math.min(...brake)).toFixed(2)} g`;
      }
    }
    if (!busy) status("ggv", changed.has("ggv") ? `設定已變更，請更新圖表。${message}` : message);
  }

  function start() {
    if (busy || !form.reportValidity()) return;
    const inputs = Object.fromEntries(Array.from(new FormData(form), ([key, value]) =>
      [key, key === "axis" ? value : Number(value)]));
    request(current, inputs);
  }

  function select(tool) {
    byId("lap-panel").hidden = tool !== "lap";
    ggvPanel.hidden = tool !== "ggv";
    for (const tab of tabs) tab.setAttribute("aria-pressed", String(tab.dataset.tool === tool));
    drawGgv();
  }

  function open(button) {
    opener = button;
    current = button.dataset.tool;
    title.textContent = titles[current];
    for (const mode of ["aero", "tire"]) {
      const controls = byId(`${mode}-controls`);
      controls.hidden = controls.disabled = mode !== current;
      byId(`${mode}-views`).hidden = mode !== current;
    }
    status(current, "");
    sample.value = 0;
    dialog.showModal();
    drawInspection();
    if (!results.has(current) || changed.has(current)) start();
  }

  for (const tab of tabs) tab.addEventListener("click", () => select(tab.dataset.tool));
  for (const button of document.querySelectorAll(".tool-shortcut")) {
    button.addEventListener("click", () => open(button));
  }
  byId("curve-close").addEventListener("click", () => dialog.close());
  dialog.addEventListener("close", () => {
    if (dialog.open) return;
    if (busy) cancel(current);
    opener?.focus();
  });
  byId("ggv-run").addEventListener("click", () => request("ggv", {}));
  form.addEventListener("submit", (event) => { event.preventDefault(); start(); });
  form.addEventListener("input", () => { changed.add(current); drawInspection(); });
  for (const id of ["aero-metric", "tire-metric", "tire-wheel", "curve-sample"]) {
    byId(id).addEventListener("input", drawInspection);
  }
  for (const id of ["ggv-view", "ggv-speed"]) byId(id).addEventListener("input", drawGgv);
  function rotate(dx, dy) {
    angle.azimuth += dx;
    angle.elevation = Math.max(-1.2, Math.min(1.2, angle.elevation + dy));
    drawGgv();
  }
  function resetAngle() {
    Object.assign(angle, defaultAngle);
    drawGgv();
  }
  const rotation = { left: [-0.17, 0], right: [0.17, 0], up: [0, 0.12], down: [0, -0.12] };
  for (const button of byId("ggv-rotation").querySelectorAll("button")) {
    const direction = button.dataset.rotate;
    button.addEventListener("click", direction === "reset" ? resetAngle : () => rotate(...rotation[direction]));
  }
  ggvCanvas.addEventListener("pointerdown", (event) => {
    if (!ggvCanvas.classList.contains("surface") || !event.isPrimary || event.button !== 0) return;
    drag = { id: event.pointerId, x: event.clientX, y: event.clientY };
    ggvCanvas.setPointerCapture(event.pointerId);
  });
  ggvCanvas.addEventListener("pointermove", (event) => {
    if (drag?.id !== event.pointerId) return;
    rotate((event.clientX - drag.x) * 0.012, (event.clientY - drag.y) * -0.012);
    drag.x = event.clientX;
    drag.y = event.clientY;
  });
  ggvCanvas.addEventListener("lostpointercapture", () => { drag = undefined; });
  ggvCanvas.addEventListener("dblclick", () => { if (ggvCanvas.classList.contains("surface")) resetAngle(); });
  new ResizeObserver(drawInspection).observe(canvas);
  new ResizeObserver(drawGgv).observe(ggvCanvas);

  return {
    select, status,
    setBusy(value) {
      busy = value;
      for (const tab of tabs) tab.disabled = value;
      byId("curve-run").disabled = value;
      byId("ggv-run").disabled = value;
      for (const input of form.querySelectorAll("input, select")) input.disabled = value;
    },
    invalidate() {
      for (const tool of results.keys()) changed.add(tool);
      drawInspection();
      drawGgv();
    },
    receive(tool, result, fileName) {
      results.set(tool, { result, fileName });
      changed.delete(tool);
      if (tool === "ggv") {
        const speed = byId("ggv-speed");
        speed.replaceChildren(new Option("代表車速疊圖", "all"), ...result.map(({ speed: kmh }, index) =>
          new Option(`${Number(kmh.toFixed(2))} km/h`, index)));
        drawGgv();
      } else {
        sample.value = 0;
        drawInspection();
      }
    },
  };
}
