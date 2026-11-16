import { loadPyodide } from "./pyodide/pyodide.mjs";

const runtimeURL = new URL("./pyodide/", import.meta.url).href;
const uploadDirectory = "/uploaded-track";
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
  pyodide.FS.mkdir(uploadDirectory);
  pyodide.runPython(`
import sys
sys.path.insert(0, "/lapsim")
from bridge import export_csv, read_settings, run_lap

last_lap = None

def simulate_browser(settings_json, path, progress):
    global last_lap
    last_lap, payload = run_lap(read_settings(settings_json, path), progress)
    return payload

def export_browser():
    return export_csv(last_lap)
`);
  return pyodide;
}

function report(message) {
  self.postMessage({ type: "progress", message });
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

    const simulate = pyodide.globals.get("simulate_browser");
    let trackPath = "";
    try {
      if (data.track) {
        const filename = data.track.name.replace(/[^\p{L}\p{N}_.-]/gu, "_");
        trackPath = `${uploadDirectory}/track-${filename}`;
        pyodide.FS.writeFile(trackPath, new Uint8Array(await data.track.arrayBuffer()));
      }
      const result = JSON.parse(simulate(JSON.stringify(data.values), trackPath, report));
      self.postMessage({ type: "result", result, trackName: data.track?.name ?? data.trackName });
    } finally {
      simulate.destroy();
      if (trackPath && pyodide.FS.analyzePath(trackPath).exists) pyodide.FS.unlink(trackPath);
    }
  } catch (error) {
    const message = String(error.message ?? error).trim().split("\n").at(-1);
    self.postMessage({ type: "error", message });
  }
};
