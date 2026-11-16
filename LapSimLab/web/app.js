import { bounds, drawChart, trackGradient } from "./charts.js";
import { installCurveTools } from "./curves.js";

const byId = (id) => document.getElementById(id);
const form = byId("settings");
const fields = byId("fields");
const run = byId("run");
const reset = byId("reset");
const cancel = byId("cancel");
const download = byId("download");
const status = byId("status");
const signal = byId("signal");
const view = byId("view");
const sample = byId("sample");
const canvas = byId("chart");
byId("legend").querySelector(".gradient").style.background = trackGradient;
const conditionalFields = Array.from(fields.querySelectorAll("[data-show]"), (row) => ({
  row,
  field: row.querySelector("input, select"),
  conditions: row.dataset.show.split(" ").map((term) => term.split("=")),
}));
const fileRows = new Map();
const inspectionInputs = {
  motor: new Set(["motor_model", "dc_voltage", "motor_path", "lp03_path"]),
  aero: new Set(["aero_model", "aero_enabled", "aero_path", "wheelbase", "front_track", "rear_track"]),
  tire: new Set(["tire_model", "load_model", "tire_path", "ellipse_path", "tire_table_path", "tire_utilization",
    "tire_radius", "tire_stiffness", "rolling_coefficient", "mass", "wheelbase", "front_track", "rear_track",
    "cg_from_front", "cg_height", "front_heave_stiffness", "front_roll_stiffness", "rear_stiffness_ratio"]),
};
const trackOnlyInputs = new Set(["track_path", "track_width", "step_m", "curvature_model", "curvature_sigma_m",
  "solver", "passes", "initial_speed", "braking_lookahead"]);
let schema;
let worker;
let inspectionWorker;
let result;
let job;
let inspectionTool;

function setStatus(message, error = false) {
  status.textContent = message;
  status.classList.toggle("error", error);
  if (job === "inspect") curves.status(inspectionTool, message, error);
}

function updateInputs() {
  for (const { row, field, conditions } of conditionalFields) {
    const visible = conditions.every(([key, accepted]) => {
      const source = form.elements[key];
      const value = source.type === "checkbox" ? String(source.checked) : source.value;
      return accepted.split(",").includes(value);
    });
    row.hidden = !visible;
    if (field) field.disabled = !visible;
  }
  for (const [key, { input, hint, clear, example }] of fileRows) {
    const selected = input.files[0];
    hint.textContent = selected ? selected.name : `Using built-in data ${schema.files[key]}`;
    clear.hidden = !selected;
    const filename = key === "aero_path" && form.elements.aero_model.value === "quadratic" ?
      schema.quadraticAeroExample : schema.files[key];
    example.href = new URL(`./inputs/${encodeURIComponent(filename)}`, import.meta.url);
    example.download = filename;
  }
}

function setupFileRows() {
  for (const key of Object.keys(schema.files)) {
    const input = form.elements[key];
    const row = input.closest(".file-input");
    const hint = document.createElement("small");
    hint.id = `${key}-hint`;
    hint.setAttribute("aria-live", "polite");
    const actions = document.createElement("div");
    actions.className = "file-tools";
    const choose = document.createElement("button");
    choose.type = "button";
    choose.className = "secondary";
    choose.textContent = "Choose File";
    choose.setAttribute("aria-label", `Choose File for ${input.labels[0].textContent.trim()}`);
    choose.setAttribute("aria-describedby", hint.id);
    choose.addEventListener("click", () => input.click());
    input.hidden = true;
    const example = document.createElement("a");
    example.textContent = "Download Example";
    const clear = document.createElement("button");
    clear.type = "button";
    clear.className = "secondary";
    clear.textContent = "Use Built-In Data";
    clear.addEventListener("click", () => {
      input.value = "";
      input.dispatchEvent(new Event("change", { bubbles: true }));
      choose.focus();
    });
    actions.append(choose, example, clear);
    row.append(actions, hint);
    fileRows.set(key, { input, hint, clear, example });
  }
}

function restoreDefaults() {
  for (const { input } of fileRows.values()) input.value = "";
  for (const [key, value] of Object.entries(schema.defaults)) {
    const field = form.elements[key];
    if (field.type === "checkbox") field.checked = value;
    else field.value = schema.percentages.includes(key) ? 100 * value : value;
  }
  updateInputs();
}

function setJob(nextJob, tool) {
  job = nextJob;
  inspectionTool = tool;
  const busy = Boolean(job);
  fields.disabled = busy;
  run.disabled = busy;
  reset.disabled = busy;
  cancel.hidden = job !== "run" && job !== "inspect";
  byId("ggv-cancel").hidden = tool !== "ggv";
  download.disabled = busy || !result;
  curves.setBusy(busy);
}

function clearResult() {
  result = undefined;
  byId("result").hidden = true;
  byId("empty").hidden = false;
  download.disabled = true;
}

function drawResult() {
  if (!result) return;
  const entry = result.signals[signal.value];
  const index = sample.valueAsNumber;
  const [min, max] = bounds(entry.values);
  byId("legend").hidden = view.value !== "track";
  byId("legend-min").textContent = `${min.toFixed(2)} ${entry.unit}`;
  byId("legend-max").textContent = `${max.toFixed(2)} ${entry.unit}`;
  byId("sample-value").textContent = `${result.distance[index].toFixed(1)} m · ${result.time[index].toFixed(2)} s · ${entry.label} ${entry.values[index].toFixed(2)} ${entry.unit}`;
  canvas.setAttribute("aria-label", `${view.selectedOptions[0].textContent} chart, ${entry.label} (${entry.unit})`);
  drawChart(canvas, result, view.value, entry, index);
}

function showResult(data, trackName) {
  result = data;
  byId("result-track").textContent = trackName;
  byId("lap-time").value = data.lapTime.toFixed(3);
  byId("top-speed").value = data.topSpeed.toFixed(1);
  byId("track-length").value = data.distance.at(-1).toFixed(1);
  byId("peak-lateral").value = data.peakLateral.toFixed(2);
  const warnings = [];
  if (data.converged === false) warnings.push("The lap solver reached the pass limit before the speed profile converged.");
  if (data.overspeedPoints) warnings.push(`${data.overspeedPoints} samples exceed the cornering speed limit.`);
  byId("warning").textContent = warnings.join(" ");
  byId("warning").hidden = !warnings.length;
  const previousSignal = signal.value || "speed_kmh";
  signal.replaceChildren(...Object.entries(data.signals).map(([key, entry]) =>
    new Option(`${entry.label} (${entry.unit})`, key)));
  signal.value = data.signals[previousSignal] ? previousSignal : "speed_kmh";
  sample.max = data.distance.length - 1;
  sample.value = 0;
  byId("empty").hidden = true;
  byId("result").hidden = false;
  drawResult();
}

function downloadCsv(contents) {
  const url = URL.createObjectURL(new Blob([contents], { type: "text/csv;charset=utf-8" }));
  const link = document.createElement("a");
  link.href = url;
  link.download = "lapsim-result.csv";
  link.click();
  window.setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function getWorker(inspect = false) {
  const existing = inspect ? inspectionWorker : worker;
  if (existing) return existing;
  const created = new Worker(new URL("./worker.js", import.meta.url), { type: "module" });
  if (inspect) inspectionWorker = created;
  else worker = created;
  created.onmessage = ({ data }) => {
    switch (data.type) {
      case "progress":
        setStatus(data.message);
        break;
      case "result":
        showResult(data.result, data.trackName);
        setStatus("Lap calculation complete.");
        setJob();
        break;
      case "inspection":
        setJob();
        curves.receive(data.tool, data.result, data.fileName);
        setStatus(data.tool === "ggv" ? "GGV calculation complete." : "Plot updated.");
        break;
      case "csv":
        downloadCsv(data.contents);
        setJob();
        setStatus("CSV ready.");
        break;
      case "error":
        setStatus(data.message, true);
        setJob();
        break;
    }
  };
  created.onerror = () => {
    created.terminate();
    if (inspect) inspectionWorker = undefined;
    else {
      worker = undefined;
      clearResult();
    }
    setStatus("The solver stopped unexpectedly. Try again.", true);
    setJob();
  };
  return created;
}

function readInputs(tool) {
  const included = inspectionInputs[tool];
  const use = (key) => included ? included.has(key) : tool !== "ggv" || !trackOnlyInputs.has(key);
  const values = {};
  for (const [key, defaultValue] of Object.entries(schema.defaults)) {
    if (!use(key)) continue;
    const field = form.elements[key];
    if (field.disabled) values[key] = defaultValue;
    else if (field.type === "checkbox") values[key] = field.checked;
    else if (field.type === "number") {
      if (!field.checkValidity()) {
        field.closest("details").open = true;
        if (included) {
          const label = field.labels[0].textContent.trim();
          curves.status(tool, `Close this window and check the ${label} settings in the input panel.`, true);
        } else {
          field.reportValidity();
        }
        return;
      }
      values[key] = schema.percentages.includes(key) ? field.valueAsNumber / 100 : field.valueAsNumber;
    } else values[key] = field.value;
  }
  const files = {};
  for (const [key, { input }] of fileRows) {
    if (use(key) && !input.disabled) files[key] = input.files[0] ?? schema.files[key];
  }
  return { values, files };
}

function startInspection(tool, conditions) {
  if (job || !schema) return;
  const inputs = readInputs(tool);
  if (!inputs) return;
  setJob("inspect", tool);
  setStatus("Preparing calculation…");
  getWorker(true).postMessage({ type: "inspect", tool, conditions, ...inputs });
}

const curves = installCurveTools(startInspection, (tool) => {
  if (inspectionTool === tool) cancelJob();
});
form.addEventListener("submit", (event) => {
  event.preventDefault();
  if (run.disabled) return;
  const inputs = readInputs();
  if (!inputs) return;
  curves.select("lap");
  clearResult();
  setJob("run");
  setStatus("Preparing calculation…");
  getWorker().postMessage({ type: "run", ...inputs });
});

form.addEventListener("reset", (event) => {
  event.preventDefault();
  restoreDefaults();
  curves.invalidate();
  clearResult();
  setStatus("Default settings restored.");
});
function cancelJob() {
  if (job === "inspect") {
    inspectionWorker.terminate();
    inspectionWorker = undefined;
  } else {
    worker.terminate();
    worker = undefined;
  }
  setStatus("Calculation canceled.");
  setJob();
}
cancel.addEventListener("click", cancelJob);
byId("ggv-cancel").addEventListener("click", cancelJob);
download.addEventListener("click", () => {
  setJob("export");
  setStatus("Generating CSV…");
  getWorker().postMessage({ type: "export" });
});
form.addEventListener("change", () => {
  updateInputs();
  curves.invalidate();
  if (result && !run.disabled) setStatus("Settings changed. Run the calculation again to update the results.");
});
for (const input of [signal, view, sample]) input.addEventListener("input", drawResult);
new ResizeObserver(drawResult).observe(canvas);

async function initialize() {
  const response = await fetch(new URL("./settings.json", import.meta.url));
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  schema = await response.json();
  for (const [key, choices] of Object.entries(schema.models)) {
    form.elements[key].replaceChildren(...Object.entries(choices).map(([value, label]) => new Option(label, value)));
  }
  setupFileRows();
  restoreDefaults();
  setJob();
  setStatus("Ready to calculate lap time.");
}

initialize().catch((error) => setStatus(`Could not load the simulation settings (${error.message}).`, true));
