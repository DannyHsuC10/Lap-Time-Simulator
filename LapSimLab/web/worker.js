import { loadPyodide } from "./pyodide/pyodide.mjs";

const runtimeURL = new URL("./pyodide/", import.meta.url).href;
const inputDirectory = "/lapsim/input_data";
const uploadDirectory = "/uploaded-inputs";
let runtime;

async function initialize() {
  const archive = fetch(new URL("./solver.zip", self.location.href)).then((response) => {
    if (!response.ok) throw new Error(`無法下載求解器（HTTP ${response.status}）。`);
    return response.arrayBuffer();
  });
  const [pyodide, bundle] = await Promise.all([
    loadPyodide({ indexURL: runtimeURL }),
    archive,
  ]);
  await pyodide.loadPackage("numpy");
  pyodide.unpackArchive(bundle, "zip", { extractDir: "/lapsim" });
  pyodide.FS.mkdir(inputDirectory);
  pyodide.FS.mkdir(uploadDirectory);
  pyodide.runPython(`
import sys
sys.path.insert(0, "/lapsim")
from bridge import export_csv, inspect_curve, read_settings, run_lap

last_lap = None

def simulate_browser(settings_json, files_json, progress):
    global last_lap
    last_lap, payload = run_lap(read_settings(settings_json, files_json), progress)
    return payload

def export_browser():
    return export_csv(last_lap)
`);
  return pyodide;
}

function report(message) {
  self.postMessage({ type: "progress", message });
}

async function prepareInputs(fs, files, uploads) {
  const paths = {};
  for (const [key, file] of Object.entries(files)) {
    if (typeof file === "string") {
      const path = `${inputDirectory}/${file}`;
      if (!fs.analyzePath(path).exists) {
        const response = await fetch(new URL(`./inputs/${encodeURIComponent(file)}`, import.meta.url));
        if (!response.ok) throw new Error(`無法載入內建資料「${file}」（HTTP ${response.status}）。`);
        fs.writeFile(path, new Uint8Array(await response.arrayBuffer()));
      }
      paths[key] = path;
    } else {
      const name = file.name.replace(/[^\p{L}\p{N}_.-]/gu, "_");
      const path = `${uploadDirectory}/${key}-${name}`;
      fs.writeFile(path, new Uint8Array(await file.arrayBuffer()));
      uploads.push(path);
      paths[key] = path;
    }
  }
  return paths;
}

self.onmessage = async ({ data }) => {
  try {
    if (!runtime) {
      report("下載 Python 求解環境…");
      runtime = initialize().catch((error) => {
        runtime = undefined;
        throw error;
      });
    }
    const pyodide = await runtime;
    if (data.type === "export") {
      self.postMessage({ type: "csv", contents: pyodide.runPython("export_browser()") });
      return;
    }

    const solve = pyodide.globals.get(data.type === "inspect" ? "inspect_curve" : "simulate_browser");
    const uploads = [];
    try {
      report("載入模型資料…");
      const paths = await prepareInputs(pyodide.FS, data.files, uploads);
      const valuesJson = JSON.stringify(data.values);
      const filesJson = JSON.stringify(paths);
      if (data.type === "inspect") {
        report(data.tool === "ggv" ? "計算 GGV 車速切片…" : "計算曲線…");
        const result = JSON.parse(solve(data.tool, valuesJson, filesJson, JSON.stringify(data.conditions)));
        const file = Object.values(data.files)[0];
        self.postMessage({ type: "inspection", tool: data.tool, result, fileName: file?.name ?? file });
        return;
      }
      const result = JSON.parse(solve(valuesJson, filesJson, report));
      const track = data.files.track_path;
      self.postMessage({ type: "result", result, trackName: typeof track === "string" ? track : track.name });
    } finally {
      solve.destroy();
      for (const path of uploads) pyodide.FS.unlink(path);
    }
  } catch (error) {
    const message = String(error.message ?? error).trim().split("\n").at(-1);
    self.postMessage({ type: "error", message });
  }
};
