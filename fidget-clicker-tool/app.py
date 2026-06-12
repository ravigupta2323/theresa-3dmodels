"""Fidget Clicker Parametric Tool - local web app.

Run:  python app.py   (or double-click run.bat)
Opens http://127.0.0.1:5723 in your browser. Everything stays on this machine.
"""

import json
import os
import tempfile
import threading
import webbrowser
from pathlib import Path

import trimesh
from flask import Flask, Response, abort, jsonify, request, send_from_directory

import pipeline

HOST, PORT = "127.0.0.1", 5723

app = Flask(__name__, static_url_path="/static", static_folder="static")
app.config["MAX_CONTENT_LENGTH"] = 512 * 1024 * 1024

STATE = {"result": None, "params": None}


@app.after_request
def no_cache(resp):
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/api/health")
def api_health():
    return jsonify({
        "ok": True,
        "has_result": STATE["result"] is not None,
        "stats": STATE["result"].stats if STATE["result"] else None,
    })


@app.get("/api/download/<part>")
def api_download(part):
    """Download the last-exported (print-pose) part as binary STL."""
    result = STATE["result"]
    if result is None or part not in ("top", "bottom"):
        abort(404)
    log = pipeline.RunLog()
    import tempfile as _tf

    with _tf.TemporaryDirectory() as td:
        paths = pipeline.export_stls(result, td, "dl", log)
        data = Path(paths[part]).read_bytes()
    return Response(data, mimetype="application/octet-stream")


@app.post("/api/run")
def api_run():
    log = pipeline.RunLog()
    tmp_path = None
    try:
        f = request.files.get("model")
        if f is None or not f.filename:
            log.error("No model file selected.")
            return jsonify({"ok": False, "log": log.entries})
        params = json.loads(request.form.get("params", "{}"))
        suffix = Path(f.filename).suffix.lower()
        fd, tmp_path = tempfile.mkstemp(suffix=suffix)
        with os.fdopen(fd, "wb") as tmp:
            f.save(tmp)
        result = pipeline.run_pipeline(tmp_path, params, log)
        STATE["result"] = result
        STATE["params"] = {**pipeline.DEFAULT_PARAMS, **params}
        return jsonify({"ok": True, "log": log.entries, "stats": result.stats})
    except pipeline.PipelineError as e:
        log.error(str(e))
        return jsonify({"ok": False, "log": log.entries})
    except Exception as e:
        log.error(f"Unexpected error: {e}")
        return jsonify({"ok": False, "log": log.entries})
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


@app.get("/api/preview/<part>")
def api_preview(part):
    """Binary STL of a preview element, in assembled pose (Z-up)."""
    result = STATE["result"]
    if result is None:
        abort(404)
    tpl = pipeline.get_templates()
    st = result.stats
    cx, cy = st["switch_center"]
    # cap drawn at its physical rest position (flush look: original position)
    rest_offset = st.get("z_cap_rest_offset", 0.0)

    if part == "bottom":
        mesh = result.part_bottom
    elif part == "top":
        mesh = result.part_top.copy()
        mesh.apply_translation([0, 0, rest_offset])
    elif part == "housing":
        mesh = tpl.housing.copy()
        mesh.apply_translation([cx, cy, st["z_house"] - tpl.housing_h])
    elif part == "keycap":
        mesh = tpl.keycap.copy()
        mesh.apply_translation(
            [cx, cy, st["z_plate"] - tpl.keycap_h + rest_offset]
        )
    else:
        abort(404)
    # keep the viewer light: decimate big meshes for PREVIEW only
    # (exports are always full resolution)
    if len(mesh.faces) > 200_000:
        try:
            dec = mesh.simplify_quadric_decimation(face_count=150_000)
            if dec is not None and len(dec.faces) > 0:
                mesh = dec
        except BaseException:
            pass
    data = trimesh.exchange.stl.export_stl(mesh)
    return Response(data, mimetype="application/octet-stream")


@app.post("/api/export")
def api_export():
    result = STATE["result"]
    log = pipeline.RunLog()
    if result is None:
        log.error("Nothing to export - run the pipeline first.")
        return jsonify({"ok": False, "log": log.entries})
    body = request.get_json(silent=True) or {}
    prefix = body.get("output_prefix") or STATE["params"].get("output_prefix", "fidget")
    output_dir = body.get("output_dir") or str(pipeline.TOOL_DIR / "output")
    try:
        paths = pipeline.export_stls(result, output_dir, prefix, log)
        return jsonify({"ok": True, "paths": paths, "log": log.entries})
    except Exception as e:
        log.error(f"Export failed: {e}")
        return jsonify({"ok": False, "log": log.entries})


def _open_browser():
    webbrowser.open(f"http://{HOST}:{PORT}")


if __name__ == "__main__":
    threading.Timer(1.0, _open_browser).start()
    print(f"Fidget Clicker Tool running at http://{HOST}:{PORT}")
    app.run(host=HOST, port=PORT, debug=False, threaded=True)
