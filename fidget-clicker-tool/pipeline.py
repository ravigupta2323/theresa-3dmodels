"""Core mesh pipeline for the Fidget Clicker Parametric Tool.

Turns a 3D model into a printable MX-switch fidget clicker:

  - TWO-PIECE inputs (two separate shells): the upper piece becomes the cap,
    the lower piece the holder - no cutting needed.
  - SINGLE-PIECE inputs: cut at a user height or at an auto-detected seam
    (the biggest step/groove in the cross-section-area profile).

Look styles:
  - "flush" (default): the switch is buried `rest_float` (~13.8mm) deep so the
    resting cap sits exactly where the original surface was - unpressed, the
    clicker looks like the original object; pressing sinks the cap into a
    hidden swallow pocket.
  - "floating": tutorial style - housing flush with the surface, cap floats
    ~13.8mm above it on the switch stem.

By default the whole model is rescaled to the SMALLEST size that still fits
the switch ("minimize").

Runs standalone for testing:
  python pipeline.py "..\\Sunny Dumpling.obj"
"""

import argparse
import collections
import json
import logging
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import manifold3d as m3d
import numpy as np
import trimesh
from shapely.geometry import Point
from shapely.geometry import box as shapely_box
from shapely.ops import unary_union

TOOL_DIR = Path(__file__).resolve().parent
TEMPLATE_DIR = TOOL_DIR.parent / "Mechanical+Switch+Keycap+Fidget+Clicker+Template+"
LOG_FILE = TOOL_DIR / "fidget_tool.log"

WALL_MARGIN_MM = 1.5      # desired body wall around the switch cavities
EMBED_MM = 0.05           # overlap used so unions are robustly connected
ANALYSIS_FACES = 150_000  # decimation target for probing huge meshes

MX_STEM_TOP_ABOVE_PLATE = 11.6  # Cherry MX datasheet
MX_TRAVEL = 4.0                 # Cherry MX total travel
SHAFT_W = 16.4            # insertion shaft: switch flange 15.6 + 0.4 per side
POCKET_DEPTH = MX_TRAVEL + 0.5  # swallow pocket below the resting cap
POCKET_CLR = 0.3          # XY clearance of the swallow pocket walls

DEFAULT_PARAMS = {
    "cap_height_mm": "auto",   # number (mm from the top) or "auto" = detect seam
    "look": "flush",           # "flush" (looks whole at rest) or "floating"
    "rest_float_mm": "auto",   # cap rest height above housing top; auto ~ 13.8
    "bottom_solid_mm": 2.2,
    "manual_scale_factor": 1.0,
    "size_mode": "minimize",   # minimize | grow_if_needed | original
    "output_prefix": "fidget",
}

logger = logging.getLogger("fidget")
if not logger.handlers:
    logger.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    fh = logging.FileHandler(LOG_FILE, encoding="utf-8")
    fh.setFormatter(fmt)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    logger.addHandler(fh)
    logger.addHandler(sh)


class RunLog:
    """Collects log entries for the UI while also writing to the logger."""

    def __init__(self):
        self.entries = []

    def _add(self, level, msg):
        self.entries.append({"level": level, "msg": msg})

    def info(self, msg):
        logger.info(msg)
        self._add("INFO", msg)

    def warn(self, msg):
        logger.warning(msg)
        self._add("WARN", msg)

    def error(self, msg):
        logger.error(msg)
        self._add("ERROR", msg)


class _SilentLog:
    def info(self, msg):
        pass

    warn = error = info


_SILENT = _SilentLog()


class PipelineError(Exception):
    pass


@dataclass
class Templates:
    negative: trimesh.Trimesh
    housing: trimesh.Trimesh
    keycap: trimesh.Trimesh
    housing_w: float
    housing_d: float
    housing_h: float
    keycap_h: float
    socket_ceiling: float   # cross-socket ceiling above the keycap opening
    rest_float: float       # cap underside height above housing top at rest


@dataclass
class PipelineResult:
    part_top: trimesh.Trimesh
    part_bottom: trimesh.Trimesh
    stats: dict = field(default_factory=dict)
    log: RunLog = None


_templates = None


# --------------------------------------------------------------------------
# manifold3d boolean engine
# --------------------------------------------------------------------------

def _mesh_to_manifold_single(mesh):
    mg = m3d.Mesh(
        vert_properties=np.asarray(mesh.vertices, dtype=np.float32),
        tri_verts=np.asarray(mesh.faces, dtype=np.uint32),
    )
    mg.merge()
    man = m3d.Manifold(mg)
    if man.status() != m3d.Error.NoError:
        raise ValueError(str(man.status()))
    return man


def _to_manifold(mesh, what, log=_SILENT):
    """Convert a Trimesh to a manifold3d Manifold. If the mesh as a whole is
    not a valid solid, fall back to converting its connected components
    individually and unioning them."""
    try:
        return _mesh_to_manifold_single(mesh)
    except ValueError:
        pass
    comps = mesh.split(only_watertight=False)
    mans, failed = [], 0
    for c in comps:
        try:
            mans.append(_mesh_to_manifold_single(c))
        except ValueError:
            # last resort: let pymeshfix rebuild the component as a clean solid
            try:
                import pymeshfix

                fixer = pymeshfix.MeshFix(
                    np.asarray(c.vertices, dtype=np.float64),
                    np.asarray(c.faces, dtype=np.int32),
                )
                fixer.repair(verbose=False)
                fixed = trimesh.Trimesh(fixer.v, fixer.f, process=False)
                mans.append(_mesh_to_manifold_single(fixed))
                log.warn(
                    f"A piece of the {what} mesh needed a deep repair "
                    "(pymeshfix) - small details may be simplified."
                )
            except Exception:
                failed += 1
    if not mans:
        raise PipelineError(
            f"Mesh for {what} is not a valid solid and none of its pieces "
            "could be repaired. Try repairing the model in Microsoft "
            "3D Builder or Meshmixer."
        )
    if failed:
        log.warn(
            f"{failed} broken piece(s) of the {what} mesh could not be "
            "repaired and were skipped."
        )
    acc = mans[0]
    for other in mans[1:]:
        acc = acc + other
    return acc


def _from_manifold(man):
    out = man.to_mesh()
    verts = np.asarray(out.vert_properties)[:, :3]
    faces = np.asarray(out.tri_verts, dtype=np.int64)
    # weld using manifold's own exact merge map (distance-based merging would
    # miss welds and leave cracks)
    merge_from = np.asarray(out.merge_from_vert, dtype=np.int64)
    merge_to = np.asarray(out.merge_to_vert, dtype=np.int64)
    if len(merge_from):
        remap = np.arange(len(verts))
        remap[merge_from] = merge_to
        while True:
            chased = remap[remap]
            if np.array_equal(chased, remap):
                break
            remap = chased
        faces = remap[faces]
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    mesh.remove_unreferenced_vertices()
    return mesh


def _boolean(op, meshes, what, log):
    try:
        t0 = time.time()
        mans = [_to_manifold(m, what, log) for m in meshes]
        acc = mans[0]
        for other in mans[1:]:
            if op == "union":
                acc = acc + other
            elif op == "difference":
                acc = acc - other
            elif op == "intersection":
                acc = acc ^ other
        result = _from_manifold(acc)
        if result.is_empty or len(result.faces) == 0:
            raise PipelineError(f"Boolean {op} ({what}) produced an empty mesh.")
        log.info(f"Boolean {op} ({what}) ok in {time.time() - t0:.1f}s")
        return result
    except PipelineError:
        raise
    except Exception as e:
        raise PipelineError(
            f"Boolean {op} failed during {what}: {e}. "
            "Try repairing the model in Microsoft 3D Builder or Meshmixer."
        )


# --------------------------------------------------------------------------
# templates
# --------------------------------------------------------------------------

def _load_template(name):
    path = TEMPLATE_DIR / name
    if not path.exists():
        raise PipelineError(f"Template not found: {path}")
    m = trimesh.load(path, force="mesh")
    m.merge_vertices()
    # normalize: XY centered on origin, base resting on z=0
    lo, hi = m.bounds
    m.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    return m


def get_templates():
    global _templates
    if _templates is None:
        negative = _load_template("SwitchHousingNegativeBlock.stl")
        housing = _load_template("SwitchHousingTemplate.stl")
        keycap = _load_template("Keycap TemplateSmaller.stl")
        # The keycap file stores the MX cross socket opening upward; flip it so
        # the socket faces down and the solid plate is on top.
        keycap.apply_transform(
            trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])
        )
        lo, hi = keycap.bounds
        keycap.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
        kh = float(keycap.extents[2])

        # how deep the cross socket reaches (probed on the keycap's axis)
        zs = np.arange(0.05, kh - 0.05, 0.05)
        inside = keycap.contains(np.column_stack([np.zeros_like(zs),
                                                  np.zeros_like(zs), zs]))
        ceiling = float(zs[np.argmax(inside)]) if inside.any() else 4.3
        rest_float = MX_STEM_TOP_ABOVE_PLATE - ceiling + kh

        size = negative.extents
        _templates = Templates(
            negative=negative,
            housing=housing,
            keycap=keycap,
            housing_w=float(size[0]),
            housing_d=float(size[1]),
            housing_h=float(size[2]),
            keycap_h=kh,
            socket_ceiling=ceiling,
            rest_float=rest_float,
        )
    return _templates


# --------------------------------------------------------------------------
# loading / cleanup
# --------------------------------------------------------------------------

def load_model(path, log):
    path = Path(path)
    ext = path.suffix.lower()
    if ext not in (".stl", ".obj"):
        raise PipelineError(
            f"Unsupported file format '{ext}'. Supported formats: .stl, .obj"
        )
    try:
        mesh = trimesh.load(path, force="mesh")
    except Exception as e:
        raise PipelineError(f"Failed to load '{path.name}': {e}")
    if mesh.is_empty or len(mesh.faces) == 0:
        raise PipelineError(f"'{path.name}' contains no triangles.")
    log.info(f"Loaded {path.name}: {len(mesh.faces):,} faces")
    return mesh


def repair(mesh, log):
    # NOTE: faces with repeated vertex indices (zero-thickness flaps from the
    # vertex merge) and exact duplicates are removed - both are safe. True
    # zero-AREA faces with distinct vertices are deliberately KEPT: deleting
    # them from a closed surface punches holes.
    mesh.merge_vertices()
    f = mesh.faces
    mesh.update_faces(
        (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    )
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    if not mesh.is_watertight:
        trimesh.repair.fill_holes(mesh)
    if mesh.is_watertight:
        log.info("Mesh is watertight after cleanup")
    else:
        log.warn(
            "Mesh is not perfectly watertight - attempting to continue "
            "(the boolean engine can absorb small cracks)."
        )
    return mesh


def _significant_components(mesh, log):
    """Split into connected components, dropping microscopic debris shells."""
    comps = mesh.split(only_watertight=False)
    if len(comps) <= 1:
        return [mesh]
    diag = float(np.linalg.norm(mesh.extents))
    keep = [c for c in comps if float(np.linalg.norm(c.extents)) >= 0.05 * diag]
    if not keep:
        keep = [max(comps, key=lambda c: len(c.faces))]
    dropped = len(comps) - len(keep)
    if dropped:
        log.info(f"Ignored {dropped} tiny debris shell(s) found in the input file")
    return keep


def _analysis_mesh(mesh):
    """A decimated copy used for cross-section probing of huge meshes."""
    if len(mesh.faces) <= ANALYSIS_FACES:
        return mesh
    try:
        dec = mesh.simplify_quadric_decimation(face_count=ANALYSIS_FACES)
        if dec is not None and len(dec.faces) > 0:
            return dec
    except BaseException:
        pass
    return mesh


# --------------------------------------------------------------------------
# geometry probing
# --------------------------------------------------------------------------

def _section_polygons(mesh, z, log=_SILENT, what=""):
    """Cross-section polygons (shapely, in world XY coords) at height z."""
    try:
        sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if sec is None:
            return []
        path2d, to_3d = sec.to_2D()
        import shapely.affinity as aff

        m = np.asarray(to_3d)
        polys = []
        for poly in path2d.polygons_full:
            if poly is None:
                continue
            moved = aff.affine_transform(
                poly, [m[0, 0], m[0, 1], m[1, 0], m[1, 1], m[0, 3], m[1, 3]]
            )
            polys.append(moved)
        return polys
    except Exception as e:
        log.warn(f"Could not compute cross-section{' for ' + what if what else ''}: {e}")
        return []


def _local_surface_z(mesh, cx, cy, half_w, half_d, from_above, n=5):
    """Lowest surface z over an XY grid spanning the switch footprint.

    Using the minimum keeps the housing (and the keycap plate) from ever
    poking out of a curved surface."""
    lo, hi = mesh.bounds
    origins, vecs = [], []
    for x in np.linspace(cx - half_w, cx + half_w, n):
        for y in np.linspace(cy - half_d, cy + half_d, n):
            if from_above:
                origins.append([x, y, hi[2] + 10.0])
                vecs.append([0, 0, -1.0])
            else:
                origins.append([x, y, lo[2] - 10.0])
                vecs.append([0, 0, 1.0])
    locations, index_ray, _ = mesh.ray.intersects_location(origins, vecs)
    if len(locations) == 0:
        return None
    # first hit per ray (= the actual surface), then the lowest across rays
    firsts = {}
    for z, ray in zip(locations[:, 2], index_ray):
        if from_above:
            firsts[ray] = max(firsts.get(ray, -np.inf), z)
        else:
            firsts[ray] = min(firsts.get(ray, np.inf), z)
    return float(min(firsts.values()))


def _find_seam(mesh, log):
    """Auto cut point: the most pronounced step or groove in the
    cross-section-area profile, preferring seams higher up the model."""
    h = float(mesh.bounds[1][2])
    n = 80
    zs = np.linspace(0.02 * h, 0.98 * h, n)
    areas = np.array([
        sum(q.area for q in _section_polygons(mesh, z)) for z in zs
    ])
    amax = areas.max()
    if amax <= 0:
        return 0.72 * h
    candidates = []
    for i in range(1, n - 1):
        frac = zs[i] / h
        if frac < 0.30 or frac > 0.88:
            continue
        step = abs(areas[i + 1] - areas[i - 1]) / amax
        groove = max(0.0, (min(areas[i - 1], areas[i + 1]) - areas[i]) / amax)
        candidates.append((max(step, groove), zs[i]))
    strong = [c for c in candidates if c[0] >= 0.06]
    if not strong:
        log.info("Auto cut: no clear seam found - cutting at 72% of the height")
        return 0.72 * h
    # among all pronounced seams, prefer the HIGHEST one (a smaller clicky
    # cap, e.g. the lid of a jar rather than a tier line lower down); within
    # that seam's cluster of samples, use its strongest point
    best = max(s for s, _ in strong)
    qualifying = sorted(
        [(s, z) for s, z in strong if s >= 0.7 * best], key=lambda t: t[1]
    )
    dz = zs[1] - zs[0]
    clusters = [[qualifying[0]]]
    for s, z in qualifying[1:]:
        if z - clusters[-1][-1][1] <= 2.5 * dz:
            clusters[-1].append((s, z))
        else:
            clusters.append([(s, z)])
    best_z = max(clusters[-1], key=lambda t: t[0])[1]
    log.info(
        f"Auto cut: detected a seam at z={best_z:.1f} "
        f"({best_z / h * 100:.0f}% of the model height)"
    )
    return float(best_z)


def _scaled_polys(mesh, z_scaled, scale):
    import shapely.affinity as aff

    polys = _section_polygons(mesh, z_scaled / scale)
    return [aff.scale(q, xfact=scale, yfact=scale, origin=(0, 0)) for q in polys]


def _cavity_probe_depths(tpl, sink):
    """(depth-below-surface, required half width) pairs for the switch
    cavities: the housing band, plus the insertion shaft band when sunk."""
    hh = tpl.housing_h
    house_half = tpl.housing_w / 2 + WALL_MARGIN_MM
    depths = [
        (sink + 0.01, house_half),
        (sink + hh / 2, house_half),
        (sink + hh - 0.01, house_half),
    ]
    if sink > 1.0:
        shaft_half = SHAFT_W / 2 + WALL_MARGIN_MM
        depths += [(1.0, shaft_half), (sink / 2, shaft_half), (sink - 0.5, shaft_half)]
    return depths


def _walls_ok(mesh, scale, cx, cy, z_top_scaled, tpl, sink):
    for depth, half in _cavity_probe_depths(tpl, sink):
        z = z_top_scaled - depth
        polys = _scaled_polys(mesh, z, scale)
        if not polys:
            return False
        fp = shapely_box(cx - half, cy - half, cx + half, cy + half)
        if not unary_union(polys).contains(fp):
            return False
    return True


def _fit_check_single(mesh, scale, tpl, bottom_solid, sink, cap_mm=None, cut_frac=None):
    """Single-piece mode: would the switch fit at uniform scale `scale`?"""
    hh = tpl.housing_h
    h_s = float(mesh.bounds[1][2]) * scale
    z_cut = cut_frac * h_s if cut_frac is not None else h_s - cap_mm
    if z_cut - sink - hh < bottom_solid:
        return False
    cut_polys = _scaled_polys(mesh, z_cut - 0.05, scale)
    if not cut_polys:
        return False
    largest = max(cut_polys, key=lambda q: q.area)
    cx, cy = largest.centroid.x, largest.centroid.y
    plate = shapely_box(cx - tpl.housing_w / 2, cy - tpl.housing_d / 2,
                        cx + tpl.housing_w / 2, cy + tpl.housing_d / 2)
    if not largest.contains(plate):
        return False
    return _walls_ok(mesh, scale, cx, cy, z_cut, tpl, sink)


def _fit_check_two(holder, cap, scale, tpl, bottom_solid, sink, cx, cy,
                   z_house_ref, z_under):
    """Two-piece mode: housing + shaft fit in the holder, and the cap can
    cover/bond to the keycap plate."""
    hh = tpl.housing_h
    z_top = z_house_ref * scale          # holder surface at the switch
    if z_top - sink - hh < bottom_solid:
        return False
    cxs, cys = cx * scale, cy * scale
    if not _walls_ok(holder, scale, cxs, cys, z_top, tpl, sink):
        return False
    # cap must be at least as wide as the keycap plate...
    c_lo, c_hi = cap.bounds
    if (c_hi[0] - c_lo[0]) * scale < tpl.housing_w or \
       (c_hi[1] - c_lo[1]) * scale < tpl.housing_d:
        return False
    # ...and reasonably cover it a couple of mm above its underside
    plate = shapely_box(cxs - tpl.housing_w / 2, cys - tpl.housing_d / 2,
                        cxs + tpl.housing_w / 2, cys + tpl.housing_d / 2)
    polys = _scaled_polys(cap, z_under * scale + 2.3, scale)
    if not polys:
        return False
    covered = unary_union(polys).intersection(plate).area
    return covered >= 0.5 * plate.area


def _search_min_scale(fit_fn, lo_guess, log):
    """Smallest uniform scale for which fit_fn(scale) holds."""
    hi = max(lo_guess, 0.001)
    tries = 0
    while not fit_fn(hi):
        hi *= 1.3
        tries += 1
        if tries > 30:
            raise PipelineError(
                "Could not find any scale at which the switch fits this model. "
                "Try a different cap height / cut point."
            )
    lo = lo_guess
    if fit_fn(lo):
        return lo * 1.005
    for _ in range(28):
        mid = (lo + hi) / 2
        if fit_fn(mid):
            hi = mid
        else:
            lo = mid
    return hi * 1.005  # small safety margin against borderline-thin walls


def _open_edge_count(mesh):
    """Boundary (hole) edges. 0 means the surface is closed and printable,
    even when trimesh's stricter is_watertight check is False."""
    cnt = collections.Counter(map(tuple, mesh.edges_sorted))
    return sum(1 for v in cnt.values() if v == 1)


def _split_islands(part, cx, cy, z_probe, log):
    """Split a top slice into the main cap component (the one over the switch
    center) and disconnected islands (e.g. tips of side features sliced off
    by the cut plane)."""
    comps = part.split(only_watertight=False)
    if len(comps) <= 1:
        return part, []
    main = None
    for c in comps:
        for poly in _section_polygons(c, z_probe, log):
            if poly.contains(Point(cx, cy)):
                main = c
                break
        if main is not None:
            break
    if main is None:
        main = max(comps, key=lambda c: abs(c.volume))
        log.warn(
            "No cap piece sits directly over the switch center - using the "
            "largest piece as the cap."
        )
    islands = [c for c in comps if c is not main]
    return main, islands


def _drop_debris(part, name, log, min_diag_mm=1.5):
    """Remove tiny floating fragments left over from boolean operations."""
    comps = part.split(only_watertight=False)
    if len(comps) <= 1:
        return part
    keep = [c for c in comps if float(np.linalg.norm(c.extents)) >= min_diag_mm]
    if not keep:
        return part
    dropped = len(comps) - len(keep)
    if dropped:
        log.info(
            f"Removed {dropped} tiny debris fragment(s) (<{min_diag_mm}mm) "
            f"from the {name}"
        )
    return keep[0] if len(keep) == 1 else trimesh.util.concatenate(keep)


def _keycap_plate_z(part_top, cx, cy, z_start, tpl, log):
    """Where the keycap plate top should sit: just above the cap underside,
    sunk deeper (max 2.3mm, keeping the cross socket clear) when the cap
    underside is curved, until the plate is fully covered."""
    plate = shapely_box(cx - tpl.housing_w / 2, cy - tpl.housing_d / 2,
                        cx + tpl.housing_w / 2, cy + tpl.housing_d / 2)
    for t in np.arange(EMBED_MM, 2.31, 0.25):
        polys = _section_polygons(part_top, z_start + t)
        if polys and unary_union(polys).contains(plate):
            if t > 0.5:
                log.info(
                    f"Keycap plate sunk {t:.2f}mm into the curved cap "
                    "underside for a solid bond"
                )
            return z_start + t
    log.warn(
        "Cap underside is very curved - the keycap plate connection is "
        "partial (sunk 2.3mm). Consider a different cut point."
    )
    return z_start + 2.3


# --------------------------------------------------------------------------
# main pipeline
# --------------------------------------------------------------------------

def run_pipeline(model_path, params, log=None):
    log = log or RunLog()
    p = {**DEFAULT_PARAMS, **(params or {})}
    tpl = get_templates()
    hw, hd, hh, kh = tpl.housing_w, tpl.housing_d, tpl.housing_h, tpl.keycap_h
    bottom_solid = float(p["bottom_solid_mm"])
    size_mode = p.get("size_mode", "minimize")
    look = p.get("look", "flush")

    try:
        rest_float = float(p.get("rest_float_mm"))
        if rest_float <= 0:
            rest_float = tpl.rest_float
    except (TypeError, ValueError):
        rest_float = tpl.rest_float
    sink = rest_float if look == "flush" else 0.0

    cap_mm = None
    try:
        cap_mm = float(p.get("cap_height_mm"))
        if cap_mm <= 0:
            cap_mm = None
    except (TypeError, ValueError):
        cap_mm = None

    log.info(
        f"Parameters: cap_height={'auto' if cap_mm is None else cap_mm} "
        f"look={look} rest_float={rest_float:.1f}mm "
        f"bottom_solid={bottom_solid}mm size_mode={size_mode}"
    )
    log.info(f"Switch preset: MX (housing {hw:.1f} x {hd:.1f} x {hh:.2f}mm)")
    if look == "flush":
        log.info(
            f"Flush look: switch buried {rest_float:.1f}mm below the seam so "
            "the unpressed clicker matches the original shape; pressing sinks "
            f"the cap up to {MX_TRAVEL:.0f}mm into a hidden pocket."
        )

    mesh = load_model(model_path, log)
    mesh = repair(mesh, log)

    # normalize: XY bbox center at origin, base on z=0
    lo, hi = mesh.bounds
    mesh.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

    total_scale = 1.0
    manual = float(p["manual_scale_factor"])
    if manual != 1.0 and size_mode != "minimize":
        log.warn(f"Manual scale factor {manual} applied")
        mesh.apply_scale(manual)
        total_scale = manual
    elif manual != 1.0:
        log.info("Minimize mode: manual scale factor is ignored")

    ext = mesh.extents
    log.info(f"Model size: {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm")

    comps = _significant_components(mesh, log)
    two_part = len(comps) == 2
    if len(comps) > 2:
        log.warn(
            f"Model has {len(comps)} separate pieces - treating them as one "
            "solid and cutting with a plane."
        )

    # ---- figure out the cut / interface and the fit function ---------------
    if two_part:
        comps = sorted(comps, key=lambda c: c.bounds.mean(axis=0)[2])
        holder_raw, cap_raw = comps
        log.info(
            "Detected a TWO-PIECE model: the upper piece becomes the cap "
            "(keycap stem added), the lower piece the switch holder."
        )
        bl = np.maximum(holder_raw.bounds[0][:2], cap_raw.bounds[0][:2])
        tr = np.minimum(holder_raw.bounds[1][:2], cap_raw.bounds[1][:2])
        if (tr <= bl).any():
            log.warn(
                "The two pieces do not overlap in X/Y - centering the switch "
                "under the upper piece."
            )
            c_lo, c_hi = cap_raw.bounds
            cx0, cy0 = (c_lo[0] + c_hi[0]) / 2, (c_lo[1] + c_hi[1]) / 2
        else:
            cx0, cy0 = (bl + tr) / 2
        z_house_ref0 = _local_surface_z(holder_raw, cx0, cy0, hw / 2, hd / 2,
                                        from_above=True)
        z_under0 = _local_surface_z(cap_raw, cx0, cy0, hw / 2, hd / 2,
                                    from_above=False)
        if z_house_ref0 is None or z_under0 is None:
            raise PipelineError(
                "Could not find the mating surfaces of the two pieces at the "
                "switch position."
            )
        holder_a, cap_a = _analysis_mesh(holder_raw), _analysis_mesh(cap_raw)

        def fit_fn(s):
            return _fit_check_two(
                holder_a, cap_a, s, tpl, bottom_solid, sink,
                cx0, cy0, z_house_ref0, z_under0
            )

        lo_guess = (sink + hh + bottom_solid) / z_house_ref0
    else:
        mesh = comps[0] if len(comps) == 1 else trimesh.util.concatenate(comps)
        mesh_a = _analysis_mesh(mesh)
        h = float(mesh.bounds[1][2])
        if cap_mm is None:
            z_seam = _find_seam(mesh_a, log)
            cut_frac = z_seam / h

            def fit_fn(s):
                return _fit_check_single(
                    mesh_a, s, tpl, bottom_solid, sink, cut_frac=cut_frac
                )

            lo_guess = (sink + hh + bottom_solid) / (cut_frac * h)
        else:
            cut_frac = None
            if cap_mm >= h:
                raise PipelineError(
                    f"cap_height_mm ({cap_mm}) is >= the model height "
                    f"({h:.1f}mm). Use a smaller cap height or 'auto'."
                )

            def fit_fn(s):
                return _fit_check_single(
                    mesh_a, s, tpl, bottom_solid, sink, cap_mm=cap_mm
                )

            lo_guess = (cap_mm + sink + hh + bottom_solid) / h

    # ---- resolve the scale --------------------------------------------------
    if size_mode == "minimize":
        s = _search_min_scale(fit_fn, lo_guess, log)
        word = "down" if s < 1 else "up"
        log.warn(
            f"Minimize mode: scaling {word} by x{s:.3f} to the smallest size "
            "that still fits the switch"
        )
    elif size_mode == "grow_if_needed":
        if fit_fn(1.0):
            s = 1.0
        else:
            s = max(1.0, _search_min_scale(fit_fn, max(lo_guess, 1.0), log))
            log.warn(f"Model too small for the switch - auto-scaled up by x{s:.3f}")
    else:
        s = 1.0
        if not fit_fn(1.0):
            log.warn(
                "The switch does not fit this model at its original size and "
                "rescaling is OFF - continuing anyway."
            )

    if s != 1.0:
        total_scale *= s
        if two_part:
            holder_raw.apply_scale(s)
            cap_raw.apply_scale(s)
            cx0, cy0 = cx0 * s, cy0 * s
            z_house_ref0, z_under0 = z_house_ref0 * s, z_under0 * s
        else:
            mesh.apply_scale(s)

    if two_part:
        combined = trimesh.util.concatenate([holder_raw, cap_raw])
        ext, bbox = combined.extents, combined.bounds
    else:
        ext, bbox = mesh.extents, mesh.bounds
    log.info(f"Final size: {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm")

    # ---- split / assign the two parts ---------------------------------------
    if two_part:
        part_top, part_bottom = cap_raw, holder_raw
        cx, cy = cx0, cy0
        z_surface = z_house_ref0   # holder surface at the switch position
        z_under = z_under0         # cap underside at the switch position
        log.info(
            f"Switch at X={cx:.2f}, Y={cy:.2f}; holder surface z={z_surface:.2f}, "
            f"cap underside z={z_under:.2f}"
        )
    else:
        h = float(mesh.bounds[1][2])
        z_cut = cut_frac * h if cut_frac is not None else h - cap_mm
        z_surface = z_cut
        z_under = z_cut
        polys = _section_polygons(mesh, z_cut - 0.05, log, "cut plane")
        if not polys:
            raise PipelineError("Model has no cross-section at the cut plane.")
        largest = max(polys, key=lambda q: q.area)
        cx, cy = largest.centroid.x, largest.centroid.y
        cut_outline = largest
        log.info(
            f"Cut plane at z={z_cut:.2f}mm; switch centered at "
            f"X={cx:.2f}, Y={cy:.2f}"
        )

        pad = 20.0
        bx, by = ext[0] + pad, ext[1] + pad
        top_box = trimesh.creation.box(extents=[bx, by, h - z_cut + pad])
        top_box.apply_translation([0, 0, z_cut + (h - z_cut + pad) / 2])
        bottom_box = trimesh.creation.box(extents=[bx, by, z_cut + pad])
        bottom_box.apply_translation([0, 0, z_cut - (z_cut + pad) / 2])
        part_top = _boolean("intersection", [mesh, top_box], "top cap cut", log)
        part_bottom = _boolean("intersection", [mesh, bottom_box], "holder cut", log)

        part_top, islands = _split_islands(part_top, cx, cy, z_cut + 0.01, log)
        if islands:
            log.warn(
                f"The cut slices through {len(islands)} side feature(s) not "
                "connected to the cap - keeping them attached to the holder."
            )
            part_bottom = _boolean(
                "union", [part_bottom] + islands, "reattaching sliced features", log
            )

    # where the keycap plate bonds to the cap (sinks into curved undersides)
    z_plate = _keycap_plate_z(part_top, cx, cy, z_under, tpl, log)

    # housing depth: at rest the plate top sits rest_float above the housing
    # top, so burying it (z_plate - rest_float) puts the cap exactly at its
    # designed position; floating look keeps the housing at the surface.
    if look == "flush":
        z_house = z_plate - rest_float
    else:
        z_house = z_surface

    # wall sanity warnings (forced placement continues regardless)
    for depth, half in _cavity_probe_depths(tpl, sink):
        z = z_surface - depth
        zpolys = _section_polygons(part_bottom, z, log)
        fp = shapely_box(cx - half, cy - half, cx + half, cy + half)
        if not zpolys or not unary_union(zpolys).contains(fp):
            log.warn(
                f"Thin/missing wall around the switch cavity at z={z:.2f}mm. "
                "Continuing with forced placement."
            )
    solid_below = z_house - hh
    if solid_below < bottom_solid - 0.05:
        log.warn(
            f"Only {solid_below:.2f}mm of solid material below the switch "
            f"housing (wanted {bottom_solid}mm)."
        )
    else:
        log.info(f"Solid material below housing: {solid_below:.2f} mm")

    # ---- holder: cavities + housing ------------------------------------------
    cutters = []
    negative = tpl.negative.copy()
    negative.apply_translation([cx, cy, z_house - hh])
    cutters.append(negative)

    top_z = float(bbox[1][2]) + 10.0
    if sink > 0 or not two_part:
        shaft_h = top_z - z_house
        shaft = trimesh.creation.box(extents=[SHAFT_W, SHAFT_W, shaft_h])
        shaft.apply_translation([cx, cy, z_house + shaft_h / 2])
        cutters.append(shaft)

    part_bottom = _boolean(
        "difference", [part_bottom] + cutters, "switch cavity + shaft", log
    )

    if sink > 0:
        # swallow pocket: room for the cap to sink MX_TRAVEL into the body
        if two_part:
            sweep = []
            for k in (1, 2, 3):
                c = part_top.copy()
                c.apply_transform(trimesh.transformations.scale_matrix(
                    1.01, [cx, cy, 0]))
                c.apply_translation([0, 0, -POCKET_DEPTH * k / 3])
                sweep.append(c)
            part_bottom = _boolean(
                "difference", [part_bottom] + sweep, "swallow pocket", log
            )
        else:
            pocket_poly = cut_outline.buffer(POCKET_CLR)
            pocket = trimesh.creation.extrude_polygon(
                pocket_poly, height=POCKET_DEPTH + 10.0
            )
            pocket.apply_translation([0, 0, z_cut - POCKET_DEPTH])
            part_bottom = _boolean(
                "difference", [part_bottom, pocket], "swallow pocket", log
            )
        log.info(
            f"Swallow pocket carved {POCKET_DEPTH:.1f}mm deep under the cap"
        )

    housing = tpl.housing.copy()
    housing.apply_translation([cx, cy, z_house - hh])
    part_bottom = _boolean("union", [part_bottom, housing], "housing insert", log)
    log.info(f"Holder: housing top at z={z_house:.2f}mm "
             f"({'buried ' + format(sink, '.1f') + 'mm' if sink else 'flush with the surface'})")

    # ---- cap: keycap connector ----------------------------------------------
    keycap = tpl.keycap.copy()
    keycap.apply_translation([cx, cy, z_plate - kh])
    part_top = _boolean("union", [part_top, keycap], "keycap connector", log)
    log.info("Cap: keycap connector fused underneath, MX socket facing down")

    part_top = _drop_debris(part_top, "cap", log)
    part_bottom = _drop_debris(part_bottom, "holder", log)
    if part_top.body_count > 1:
        log.warn(f"Cap has {part_top.body_count} disconnected bodies")
    if part_bottom.body_count > 1:
        log.warn(f"Holder has {part_bottom.body_count} disconnected bodies")

    for name, part in (("cap", part_top), ("holder", part_bottom)):
        open_edges = _open_edge_count(part)
        if open_edges:
            log.warn(f"The {name} part has {open_edges} open edges (holes)")
        else:
            log.info(f"The {name} part is a closed, printable solid")

    # how far from its designed position the cap sits at rest (flush: ~0)
    rest_offset = (z_house + rest_float) - z_plate

    stats = {
        "mode": "two_piece" if two_part else "single_piece",
        "look": look,
        "model_size_mm": [round(float(v), 2) for v in ext],
        "cap_size_mm": [round(float(v), 2) for v in part_top.extents],
        "holder_size_mm": [round(float(v), 2) for v in part_bottom.extents],
        "scale_applied": round(total_scale, 4),
        "z_cut": round(float(z_surface), 2),
        "z_plate": round(float(z_plate), 2),
        "z_house": round(float(z_house), 2),
        "z_under_design": round(float(z_under), 2),
        "z_cap_rest_offset": round(float(rest_offset), 2),
        "rest_float_mm": round(float(rest_float), 2),
        "travel_mm": MX_TRAVEL,
        "switch_center": [round(float(cx), 2), round(float(cy), 2)],
        "housing_size": [hw, hd, round(hh, 2)],
        "solid_below_housing_mm": round(float(solid_below), 2),
        "cap_faces": len(part_top.faces),
        "holder_faces": len(part_bottom.faces),
        "bbox": [
            [round(float(v), 2) for v in bbox[0]],
            [round(float(v), 2) for v in bbox[1]],
        ],
    }
    log.info("Pipeline finished")
    return PipelineResult(part_top=part_top, part_bottom=part_bottom, stats=stats, log=log)


def export_stls(result, output_dir, prefix, log):
    """Write both parts as binary STL, reoriented flat for printing."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    bottom = result.part_bottom.copy()
    lo, hi = bottom.bounds
    bottom.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

    top = result.part_top.copy()
    top.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    lo, hi = top.bounds
    top.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

    paths = {}
    for name, mesh_ in (("top", top), ("bottom", bottom)):
        path = out / f"{prefix}_{name}.stl"
        mesh_.export(path)
        paths[name] = str(path.resolve())
        log.info(f"Exported {path.resolve()} ({len(mesh_.faces):,} faces)")
    return paths


def main():
    ap = argparse.ArgumentParser(description="Fidget clicker pipeline (CLI test mode)")
    ap.add_argument("model")
    ap.add_argument("--cap-height", default="auto",
                    help="mm from the top, or 'auto' to detect the seam")
    ap.add_argument("--look", choices=["flush", "floating"], default="flush")
    ap.add_argument("--rest-float", default="auto")
    ap.add_argument("--bottom-solid", type=float, default=DEFAULT_PARAMS["bottom_solid_mm"])
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument(
        "--size-mode",
        choices=["original", "grow_if_needed", "minimize"],
        default="minimize",
    )
    ap.add_argument("--out", default=str(TOOL_DIR / "output"))
    ap.add_argument("--prefix", default="fidget")
    args = ap.parse_args()

    log = RunLog()
    try:
        result = run_pipeline(
            args.model,
            {
                "cap_height_mm": args.cap_height,
                "look": args.look,
                "rest_float_mm": args.rest_float,
                "bottom_solid_mm": args.bottom_solid,
                "manual_scale_factor": args.scale,
                "size_mode": args.size_mode,
            },
            log,
        )
        export_stls(result, args.out, args.prefix, log)
        print(json.dumps(result.stats, indent=2))
    except PipelineError as e:
        log.error(str(e))
        sys.exit(1)


if __name__ == "__main__":
    main()
