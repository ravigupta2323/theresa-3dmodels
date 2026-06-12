"""End-to-end self test against the running app (http://127.0.0.1:5723).

Uploads a model through the same API the browser uses, prints the pipeline
log, exports, downloads both STLs back and validates them.

  python selftest.py "..\\Sunny Dumpling.obj"
  python selftest.py "model.stl" --params "{\"look\": \"floating\"}"
"""

import argparse
import io
import json
import sys
import urllib.request

import trimesh

BASE = "http://127.0.0.1:5723"


def post_multipart(url, file_path, params):
    import mimetypes
    import uuid

    boundary = uuid.uuid4().hex
    with open(file_path, "rb") as f:
        file_data = f.read()
    name = file_path.replace("\\", "/").split("/")[-1]
    body = io.BytesIO()
    for field, value in [("params", json.dumps(params))]:
        body.write(f"--{boundary}\r\nContent-Disposition: form-data; "
                   f"name=\"{field}\"\r\n\r\n{value}\r\n".encode())
    body.write(f"--{boundary}\r\nContent-Disposition: form-data; "
               f"name=\"model\"; filename=\"{name}\"\r\n"
               "Content-Type: application/octet-stream\r\n\r\n".encode())
    body.write(file_data)
    body.write(f"\r\n--{boundary}--\r\n".encode())
    req = urllib.request.Request(
        url, data=body.getvalue(),
        headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
    )
    with urllib.request.urlopen(req, timeout=600) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("--params", default="{}", help="JSON overriding defaults")
    args = ap.parse_args()
    params = json.loads(args.params)

    print(f"== uploading {args.model} to {BASE}/api/run")
    data = post_multipart(f"{BASE}/api/run", args.model, params)
    for e in data.get("log", []):
        print(f"  [{e['level']}] {e['msg']}")
    if not data.get("ok"):
        print("RUN FAILED")
        sys.exit(1)
    stats = data["stats"]
    print(f"== stats: mode={stats['mode']} look={stats['look']} "
          f"size={stats['model_size_mm']} scale=x{stats['scale_applied']} "
          f"rest_offset={stats['z_cap_rest_offset']}")

    failures = []
    for part in ("top", "bottom"):
        with urllib.request.urlopen(f"{BASE}/api/download/{part}", timeout=600) as r:
            blob = r.read()
        m = trimesh.load(io.BytesIO(blob), file_type="stl", force="mesh")
        m.merge_vertices()
        import collections

        cnt = collections.Counter(map(tuple, m.edges_sorted))
        open_edges = sum(1 for v in cnt.values() if v == 1)
        size = (m.bounds[1] - m.bounds[0]).round(2)
        base_z = round(float(m.bounds[0][2]), 3)
        status = "OK" if open_edges == 0 and base_z == 0 else "FAIL"
        if status == "FAIL":
            failures.append(part)
        print(f"== {part}: {len(m.faces):,} faces, size {size} mm, "
              f"open edges {open_edges}, sits on z=0: {base_z == 0} -> {status}")

    if failures:
        print(f"SELF TEST FAILED: {failures}")
        sys.exit(1)
    print("SELF TEST PASSED")


if __name__ == "__main__":
    main()
