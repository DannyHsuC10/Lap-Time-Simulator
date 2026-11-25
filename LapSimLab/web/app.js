import { bounds, drawChart, trackGradient } from "./charts.js";
import { installCurveTools } from "./curves.js";

const byId = (id) => document.getElementById(id);
const form = byId("settings");
const fields = byId("fields");
const run = byId("run");
const reset = byId("reset");
const importJson = byId("import-json");
const exportJson = byId("export-json");
const projectFile = byId("project-file");
const vehicleSettingsFile = byId("vehicle-settings-file");
const cancel = byId("cancel");
const download = byId("download");
const status = byId("status");
const signal = byId("signal");
const view = byId("view");
const notesTopic = byId("model-notes-topic");
const notesContent = byId("model-notes-content");
const sample = byId("sample");
const canvas = byId("chart");
byId("legend").querySelector(".gradient").style.background = trackGradient;
const conditionalFields = Array.from(fields.querySelectorAll("[data-show]"), (row) => ({
  row,
  field: row.querySelector("input, select"),
  conditions: row.dataset.show.split(" ").map((term) => term.split("=")),
}));
const fileRows = new Map();
const projectFormat = "LapSimLab-Web";
const projectVersion = 1;
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
  drawModelNotes();
}

function drawModelNotes() {
  const key = notesTopic.value;
  const variant = key === "aero_model" && !form.elements.aero_enabled.checked ? "disabled" : form.elements[key].value;
  const id = `reference-${key}-${variant}`;
  if (notesContent.dataset.reference === id) return;
  notesContent.replaceChildren(byId(id).content.cloneNode(true));
  notesContent.dataset.reference = id;
  notesContent.scrollTop = 0;
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

function applySettings(values, { displayValues = false } = {}) {
  for (const [key, value] of Object.entries(values)) {
    const field = form.elements[key];
    if (field.type === "checkbox") field.checked = value;
    else field.value = !displayValues && schema.percentages.includes(key) ? 100 * value : value;
  }
  updateInputs();
}

function restoreDefaults() {
  for (const { input } of fileRows.values()) input.value = "";
  applySettings(schema.defaults);
}

function encodeFile(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve({
      name: file.name,
      base64: reader.result.slice(reader.result.indexOf(",") + 1),
    });
    reader.onerror = () => reject(reader.error);
    reader.readAsDataURL(file);
  });
}

async function snapshotProject() {
  const settings = Object.fromEntries(Object.keys(schema.defaults).map((key) => {
    const field = form.elements[key];
    return [key, field.type === "checkbox" ? field.checked : field.value];
  }));
  const files = {};
  for (const [key, { input }] of fileRows) {
    const file = input.files[0];
    if (file) files[key] = await encodeFile(file);
  }
  return { application: projectFormat, schema_version: projectVersion, settings, files };
}

function readProject(project) {
  const isObject = (value) => value !== null && typeof value === "object" && !Array.isArray(value);
  if (project?.application !== projectFormat || !isObject(project.settings) || !isObject(project.files)) {
    throw new Error("Choose a LapSimLab web settings export.");
  }
  if (project.schema_version !== projectVersion) throw new Error("Unsupported settings file version.");

  const settings = {};
  for (const key of Object.keys(schema.defaults)) {
    const field = form.elements[key];
    const value = project.settings[key];
    const expected = field.type === "checkbox" ? "boolean" : "string";
    if (typeof value !== expected) throw new Error(`Invalid or missing setting: ${key}.`);
    if (field.type === "number") {
      const probe = document.createElement("input");
      probe.type = "number";
      probe.value = value;
      if (probe.value !== value) throw new Error(`Invalid number: ${key}.`);
    }
    if (Object.hasOwn(schema.models, key) && !Object.hasOwn(schema.models[key], value)) {
      throw new Error(`Unknown model option: ${key}.`);
    }
    settings[key] = value;
  }

  const files = new Map();
  for (const [key, entry] of Object.entries(project.files)) {
    if (!fileRows.has(key) || !isObject(entry) || typeof entry.name !== "string" ||
        !entry.name || typeof entry.base64 !== "string") {
      throw new Error(`Invalid attachment: ${key}.`);
    }
    let bytes;
    try {
      bytes = Uint8Array.from(atob(entry.base64), (character) => character.charCodeAt(0));
    } catch {
      throw new Error(`Could not decode attachment: ${key}.`);
    }
    const transfer = new DataTransfer();
    transfer.items.add(new File([bytes], entry.name));
    files.set(key, transfer.files);
  }
  return { settings, files };
}

exportJson.addEventListener("click", async () => {
  if (job || !schema) return;
  setJob("save-project");
  try {
    const contents = JSON.stringify(await snapshotProject(), null, 2) + "\n";
    downloadFile(contents, "application/json", "lapsimlab-settings.json");
    setStatus("Settings JSON exported.");
  } catch (error) {
    setStatus(`Could not export settings. ${error.message}`, true);
  } finally {
    setJob();
  }
});

importJson.addEventListener("click", () => projectFile.click());
projectFile.addEventListener("change", async () => {
  const file = projectFile.files[0];
  projectFile.value = "";
  if (!file || job || !schema) return;
  setJob("load-project");
  try {
    const { settings, files } = readProject(JSON.parse(await file.text()));
    for (const [key, { input }] of fileRows) {
      if (files.has(key)) input.files = files.get(key);
      else input.value = "";
    }
    applySettings(settings, { displayValues: true });
    curves.invalidate();
    clearResult();
    setStatus(`Imported ${file.name}. Calculate the lap to see results for these settings.`);
  } catch (error) {
    const message = error instanceof SyntaxError ? "The file does not contain valid JSON." : error.message;
    setStatus(`Could not import settings. ${message}`, true);
  } finally {
    setJob();
  }
});

function vehicleSettings(project) {
  if (project?.application !== "VehicleInitialSizingLab-DynamicsExport" || ![2, 3].includes(project.schema_version)) {
    throw new Error("Choose a Vehicle Initial Sizing Dynamics export (version 2 or 3).");
  }
  const { vehicle, dynamics, schema_version: version, source_project: source } = project;
  function number(value, key, allowZero = false, max = Infinity) {
    if (!Number.isFinite(value) || value < 0 || (!allowZero && value === 0) || value > max) {
      throw new Error(`Invalid or missing value for ${key}.`);
    }
    return value;
  }
  function rate(channel, scale) {
    const stiffness = number(dynamics?.[`${channel}_k`], `dynamics.${channel}_k`);
    const motionRatio = number(dynamics?.[`${channel}_mr`], `dynamics.${channel}_mr`);
    return number(scale * stiffness * motionRatio ** 2, `${channel} rate`);
  }

  const lf = number(vehicle?.lf, "vehicle.lf");
  const lr = number(vehicle?.lr, "vehicle.lr");
  const mass = number(vehicle?.m, "vehicle.m");
  const wheelbase = number(lf + lr, "wheelbase");
  const cgHeight = number(vehicle?.h_cg, "vehicle.h_cg", true);
  // Version 2 heave rates used the sum of two wheel displacements. Version 3 exports the main spring rate.
  const heaveScale = version === 2 ? 4 : 1;
  const frontHeave = rate("heave_f", heaveScale);
  const settings = {
    load_model: "point_mass",
    brake_model: "leverage",
    tire_utilization: schema.defaults.tire_utilization,
    mass,
    wheelbase,
    cg_from_front: lf,
    cg_height: cgHeight,
    front_track: number(vehicle?.tf, "vehicle.tf"),
    rear_track: number(vehicle?.tr, "vehicle.tr"),
    front_heave_stiffness: frontHeave,
    // The export's roll rate acts across the full track. LapSim uses half the track in its roll equation.
    front_roll_stiffness: rate("roll_f", 4),
    rear_stiffness_ratio: number(rate("heave_r", heaveScale) / frontHeave, "rear-to-front stiffness ratio"),
    tire_stiffness: rate("tire_f", 1) / 2 + rate("tire_r", 1) / 2,
    front_brake_bias: number(vehicle?.brake_front, "vehicle.brake_front", true, 1),
  };
  if (version === 3 || source !== undefined) {
    const p = source?.parameters;
    settings.pedal_force = number(p?.brake_driver_force_N, "source_project.parameters.brake_driver_force_N");
    const target = number(p?.brake_ax ?? p?.ax, "source_project brake deceleration target");
    const gravity = number(vehicle?.gravity, "vehicle.gravity");
    // Sizing defines mechanical advantage using the requested total braking force on level ground.
    settings.brake_leverage = number(mass * gravity * target / settings.pedal_force, "mechanical advantage");

    const accelerationG = number(p?.a_acc, "source_project.parameters.a_acc");
    const radius = number(vehicle?.free_radius, "vehicle.free_radius");
    const motorTorque = number(p?.drive_motor_torque_Nm, "source_project.parameters.drive_motor_torque_Nm");
    const motorRpm = number(p?.drive_motor_rpm, "source_project.parameters.drive_motor_rpm");
    const targetSpeed = number(p?.drive_target_speed_kmh, "source_project.parameters.drive_target_speed_kmh") / 3.6;
    const rearLoadFraction = (lf + accelerationG * cgHeight) / wheelbase;
    if (!(rearLoadFraction > 0 && rearLoadFraction < 1)) {
      throw new Error("The target acceleration must keep both axles loaded to calculate the gear ratio.");
    }
    const maxWheelForce = mass * gravity * accelerationG * Math.max(rearLoadFraction, 1 - rearLoadFraction) / 2;
    const accelerationRatio = maxWheelForce * radius / motorTorque;
    const topSpeedRatio = motorRpm / (targetSpeed / (2 * Math.PI * radius) * 60);
    settings.gear_ratio = number((accelerationRatio + topSpeedRatio) / 2, "fixed gear reduction ratio");
  }
  return settings;
}

byId("vehicle-import").addEventListener("click", () => vehicleSettingsFile.click());
vehicleSettingsFile.addEventListener("change", async () => {
  const file = vehicleSettingsFile.files[0];
  vehicleSettingsFile.value = "";
  if (!file || job || !schema) return;
  setJob("import");
  try {
    const text = await file.text();
    const settings = vehicleSettings(JSON.parse(text));
    notesTopic.value = "load_model";
    applySettings(settings);
    for (const key of ["load_model", "brake_model"]) form.elements[key].closest("details").open = true;
    if (Object.hasOwn(settings, "gear_ratio")) form.elements.gear_ratio.closest("details").open = true;
    curves.invalidate();
    const sourceMessage = Object.hasOwn(settings, "pedal_force") ? "" : " This export lacks pedal force and gear sizing inputs.";
    const resultMessage = result ? " Run the calculation again to update the results." : "";
    setStatus(`Imported ${file.name}.${sourceMessage}${resultMessage}`);
  } catch (error) {
    const message = error instanceof SyntaxError ? "The file does not contain valid JSON." : error.message;
    setStatus(`Could not import vehicle settings. ${message}`, true);
  } finally {
    setJob();
  }
});

function setJob(nextJob, tool) {
  job = nextJob;
  inspectionTool = tool;
  const busy = Boolean(job);
  fields.disabled = busy;
  run.disabled = busy;
  reset.disabled = busy;
  importJson.disabled = busy;
  exportJson.disabled = busy;
  cancel.hidden = job !== "run" && job !== "inspect";
  byId("ggv-cancel").hidden = tool !== "ggv";
  download.disabled = busy || !result;
  curves.setBusy(busy);
}

function clearResult() {
  result = undefined;
  curves.setLapResult();
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
  curves.setLapResult(data);
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

function downloadFile(contents, type, filename) {
  const url = URL.createObjectURL(new Blob([contents], { type }));
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
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
        downloadFile(data.contents, "text/csv;charset=utf-8", "lapsim-result.csv");
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
form.addEventListener("change", (event) => {
  if (event.target === vehicleSettingsFile) return;
  const field = event.target.name;
  const topic = field === "aero_enabled" ? "aero_model" : field;
  if (Object.hasOwn(schema.models, topic)) notesTopic.value = topic;
  updateInputs();
  curves.invalidate();
  if (result && !run.disabled) setStatus("Settings changed. Run the calculation again to update the results.");
});
notesTopic.addEventListener("change", drawModelNotes);
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
  for (const select of fields.querySelectorAll("select")) {
    if (Object.hasOwn(schema.models, select.name)) {
      const title = select.closest("details").querySelector("summary").textContent.trim();
      notesTopic.add(new Option(title, select.name));
    }
  }
  restoreDefaults();
  setJob();
  setStatus("Ready to calculate lap time.");
}

initialize().catch((error) => setStatus(`Could not load the simulation settings (${error.message}).`, true));
