"""Package the browser app and the shared LapSimLab solver for static hosting."""

import argparse
from functools import partial
from hashlib import sha256
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
from pathlib import Path
import shutil
import sys
import tarfile
from urllib.request import urlopen
from zipfile import ZIP_DEFLATED, ZipFile


WEB = Path(__file__).resolve().parent
PROJECT = WEB.parent
sys.path.insert(0, str(PROJECT))

from lap_sim_lab.settings import MODEL_CHOICES, Settings


MODULES = (
    "__init__", "aero", "brake", "car", "motor", "planner", "settings",
    "signals", "simulation", "suspension", "tire", "track",
)
MODEL_FIELDS = ("solver", "load_model")
DATA_FIELDS = ("track_path", "tire_path", "motor_path", "aero_path")
NUMBER_FIELDS = ("mass", "max_speed", "step_m", "track_width", "passes", "initial_speed", "tire_utilization")
PYODIDE_VERSION = "314.0.8"
PYODIDE_SHA256 = "ff3e5f9d9bc8e98ff30b99515535fc24d375994f47a978c61975d653e162ca37"
PYODIDE_FILES = ("pyodide.mjs", "pyodide.asm.mjs", "pyodide.asm.wasm", "pyodide-lock.json", "python_stdlib.zip")


def download(url, expected_digest=None):
    with urlopen(url, timeout=45) as response:
        contents = response.read()
    if expected_digest and sha256(contents).hexdigest() != expected_digest:
        raise ValueError(f"Unexpected SHA-256 digest for {url}")
    return contents


def build_runtime(destination):
    version = PYODIDE_VERSION
    github = "https://github.com/pyodide/pyodide/releases/download"
    core = download(f"{github}/{version}/pyodide-core-{version}.tar.bz2", PYODIDE_SHA256)
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=BytesIO(core), mode="r:bz2") as bundle:
        for name in PYODIDE_FILES:
            with bundle.extractfile(f"pyodide/{name}") as source, (destination / name).open("wb") as target:
                shutil.copyfileobj(source, target)

    lock = json.loads((destination / "pyodide-lock.json").read_bytes())
    numpy = lock["packages"]["numpy"]
    if numpy["depends"]:
        raise ValueError("Update the Pages build to include NumPy's Pyodide dependencies.")
    wheel = numpy["file_name"]
    cdn = f"https://cdn.jsdelivr.net/pyodide/v{version}/full/"
    (destination / wheel).write_bytes(download(cdn + wheel, numpy["sha256"]))

    source_cdn = "https://cdn.jsdelivr.net/gh/"
    python_version = lock["info"]["python"]
    (destination / "LICENSE-Pyodide.txt").write_bytes(download(f"{source_cdn}pyodide/pyodide@{version}/LICENSE"))
    (destination / "LICENSE-Python.txt").write_bytes(download(f"{source_cdn}python/cpython@v{python_version}/LICENSE"))


def build(destination):
    defaults = Settings()
    values = {key: getattr(defaults, key) for key in (*MODEL_FIELDS, *NUMBER_FIELDS, "aero_enabled")}
    data_files = {getattr(defaults, key) for key in DATA_FIELDS}

    destination.mkdir(parents=True, exist_ok=True)
    for name in ("index.html", "style.css", "app.js", "worker.js", "charts.js"):
        shutil.copyfile(WEB / name, destination / name)
    schema = {
        "defaults": values,
        "models": {key: MODEL_CHOICES[key][1] for key in MODEL_FIELDS},
        "track": defaults.track_path.name,
    }
    (destination / "settings.json").write_text(json.dumps(schema, ensure_ascii=False), encoding="utf-8")
    with ZipFile(destination / "solver.zip", "w", ZIP_DEFLATED) as bundle:
        for name in MODULES:
            path = Path("lap_sim_lab") / f"{name}.py"
            bundle.write(PROJECT / path, path.as_posix())
        bundle.write(WEB / "bridge.py", "bridge.py")
        for source in sorted(data_files):
            bundle.write(source, f"input_data/{source.name}")
    build_runtime(destination / "pyodide")


class PreviewHandler(SimpleHTTPRequestHandler):
    extensions_map = SimpleHTTPRequestHandler.extensions_map | {
        ".mjs": "text/javascript", ".wasm": "application/wasm",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--serve", action="store_true", help="Serve the build on an available local port")
    args = parser.parse_args()
    build(args.destination)
    if args.serve:
        handler = partial(PreviewHandler, directory=str(args.destination))
        with ThreadingHTTPServer(("127.0.0.1", 0), handler) as server:
            print(f"LapSimLab preview at http://127.0.0.1:{server.server_port}", flush=True)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
