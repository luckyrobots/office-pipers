#!/usr/bin/env python3
"""Generates procedural-table.hpodgraph (Lucky Engine pod graph, compact form).

Everything random is a pure function of the exposed `seed` (+ per-aspect shuffle counters),
built from `expr:` parameters (sin/floor hash), so it needs no random node and re-rolls live in the editor.
"""
import math
import re
import sys
from collections import OrderedDict, defaultdict

PARAMS = OrderedDict()
NODES = []
LINKS = []
_next = [1]
_k = [0]
SECTION = ["Setup"]
SECTION_ORDER = []
FUNCS = {"min", "max", "abs", "floor", "ceil", "round", "sin", "cos", "tan", "pow", "sqrt", "pi"}


def fmt(x):
    if isinstance(x, bool):
        return "1" if x else "0"
    if isinstance(x, int):
        return str(x)
    s = f"{x:.6g}"
    return s


# ---- input value specs ------------------------------------------------------
def P(n): return ("param", n)
def F(v): return ("Float", float(v))
def I(v): return ("Int32", int(v))
def S(v): return ("String", v)
def M(v): return ("Menu", v)
def V3(a, b, c): return ("Float3", (float(a), float(b), float(c)))
def MAT(l): return ("Material", tuple(float(x) for x in l))
def v(x): return P(x) if isinstance(x, str) else F(x)


# ---- parameters -------------------------------------------------------------
def lit(name, kind, default, desc, group=None, options=None, mn=None, mx=None):
    assert name not in PARAMS, name
    d = OrderedDict(kind=kind, default=default)
    if group: d["group"] = group
    if options: d["options"] = options
    if mn is not None: d["min"] = mn
    if mx is not None: d["max"] = mx
    d["desc"] = desc
    PARAMS[name] = d
    return name


def ex(name, expr):
    assert name not in PARAMS, f"duplicate param {name}"
    names = set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", expr)) - FUNCS
    for n in names:
        assert n in PARAMS, f"{name}: expr uses undeclared '{n}': {expr}"
    assert len(names) <= 8, f"{name}: {len(names)} variables (>8 falls back to a Math tree): {expr}"
    PARAMS[name] = OrderedDict(kind="Float", expr=expr)
    return name


def rnd(name, seedvar):
    """Uniform [0,1) hash of (aspect seed, unique salt)."""
    _k[0] += 1
    c = fmt(_k[0] * 78.233)
    h = f"sin({seedvar}*12.9898+{c})*43758.5453"
    return ex("r_" + name, f"{h} - floor({h})")


def by_type(vals):
    terms = [f"is{t}*{fmt(x)}" for t, x in enumerate(vals) if x != 0]
    return " + ".join(terms) if terms else "0"


def lerp1(name, r, lo, hi):
    return ex(name, f"{fmt(lo)} + {r}*{fmt(hi - lo)}")


def lerp_t(name, r, ranges):
    los = [a for a, b in ranges]
    spans = [b - a for a, b in ranges]
    if len(set(ranges)) == 1:
        return lerp1(name, r, los[0], los[0] + spans[0])
    return ex(name, f"({by_type(los)}) + {r}*({by_type(spans)})")


def flag(name, r, probs):
    if isinstance(probs, (int, float)):
        return ex(name, f"floor({r} + {fmt(probs)})")
    return ex(name, f"floor({r} + {by_type(probs)})")


# ---- nodes ------------------------------------------------------------------
def N(typ, name, inputs=None):
    nid = _next[0]
    _next[0] += 1
    if SECTION[0] not in SECTION_ORDER:
        SECTION_ORDER.append(SECTION[0])
    NODES.append(dict(id=nid, type=typ, name=name, inputs=OrderedDict(inputs or {}), section=SECTION[0]))
    return nid


def L(a, apin, b, bpin):
    LINKS.append((a, apin, b, bpin))


_cmp = {}


def compare(param, op, value):
    key = (param, op, value)
    if key not in _cmp:
        saved = SECTION[0]
        SECTION[0] = "Conditions"
        _cmp[key] = N("Compare", f"{param} {op} {fmt(value)}", {"A": P(param), "B": F(value), "Operation": M(op)})
        SECTION[0] = saved
    return _cmp[key]


def gate(geo, cond_param, name, op="Greater", value=0.5):
    """Geometry passes only when the (0/1) parameter satisfies the comparison."""
    c = compare(cond_param, op, value)
    sw = N("Switch", name, {})
    L(c, "Result", sw, "Condition")
    L(geo, "Geometry", sw, "B")
    return sw


def variant(cond_param, a_geo, b_geo, name):
    """Condition false -> A, true -> B."""
    c = compare(cond_param, "Greater", 0.5)
    sw = N("Switch", name, {})
    L(c, "Result", sw, "Condition")
    L(a_geo, "Geometry", sw, "A")
    L(b_geo, "Geometry", sw, "B")
    return sw


def join_all(ids, name):
    ids = list(ids)
    while len(ids) > 1:
        nxt = []
        for i in range(0, len(ids), 2):
            if i + 1 < len(ids):
                j = N("JoinGeometry", name, {})
                L(ids[i], "Geometry", j, "A")
                L(ids[i + 1], "Geometry", j, "B")
                nxt.append(j)
            else:
                nxt.append(ids[i])
        ids = nxt
    return ids[0]


def position_input(node_inputs, pos, label):
    """Returns the CombineXYZ node id (or fills a literal) for a translation."""
    if all(not isinstance(c, str) for c in pos):
        node_inputs["Translation"] = V3(*pos)
        return None
    cx = N("CombineXYZ", label + " translation", {"X": v(pos[0]), "Y": v(pos[1]), "Z": v(pos[2])})
    return cx


def bevel_param(slug, dims):
    def s(d): return d if isinstance(d, str) else fmt(d)
    m = s(dims[0])
    for d in dims[1:]:
        m = f"min({m},{s(d)})"
    return ex("bv_" + slug, f"max(0, min(bevel_eff, 0.4*{m}))")


def place(geo, pos, name):
    inputs = OrderedDict()
    cx = position_input(inputs, pos, name)
    t = N("Transform", name, inputs)
    L(geo, "Geometry", t, "Geometry")
    if cx is not None:
        L(cx, "Vector", t, "Translation")
    return t


def part(name, shape, dims, mat, bevel_dims=None, sides=24, pos=None):
    """cube/cylinder -> bevel -> [place] -> material -> mesh name. `mat` is (node, pin)."""
    slug = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    if shape == "cube":
        sx, sy, sz = dims
        shp = N("MeshCube", name, {"SizeX": v(sx), "SizeY": v(sy), "SizeZ": v(sz)})
        bd = bevel_dims or [sx, sy, sz]
    else:
        r, h = dims
        shp = N("MeshCylinder", name, {"Sides": I(sides), "Radius": v(r), "Height": v(h)})
        bd = bevel_dims or [r, h]
    bv = bevel_param(slug, bd)
    bev = N("BevelEdge", name + " bevel", {"Offset": P(bv), "Segments": P("bevel_segments")})
    L(shp, "Geometry", bev, "Geometry")
    cur = bev
    if pos is not None:
        cur = place(cur, pos, name + " position")
    sm = N("SetMaterial", name + " finish", {"Name": S(name)})
    L(cur, "Geometry", sm, "Geometry")
    L(mat[0], mat[1], sm, "Material")
    nm = N("SetMeshName", name, {"Name": S(name)})
    L(sm, "Geometry", nm, "Geometry")
    return nm


# ---- palette ----------------------------------------------------------------
def lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def hexlin(h, metal, rough):
    h = h.lstrip("#")
    r, g, b = (int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
    return [round(lin(r), 4), round(lin(g), 4), round(lin(b), 4), 1.0, metal, rough]


PALETTE = [
    ("Light oak",         "#C9A878", 0.0, 0.55),  # 0
    ("Natural birch",     "#E3CFA4", 0.0, 0.50),  # 1
    ("Walnut",            "#5E3B26", 0.0, 0.50),  # 2
    ("Dark ash",          "#3A2A20", 0.0, 0.50),  # 3
    ("Pine",              "#D9B77A", 0.0, 0.60),  # 4
    ("White laminate",    "#E8EAEB", 0.0, 0.38),  # 5
    ("Black laminate",    "#1B1D1F", 0.0, 0.40),  # 6
    ("Concrete grey",     "#8A8C8D", 0.0, 0.85),  # 7
    ("Grey laminate",     "#A7ABAE", 0.0, 0.40),  # 8
    ("Cherry",            "#8C4A2F", 0.0, 0.50),  # 9
    ("Sage paint",        "#8FA58C", 0.0, 0.55),  # 10
    ("Navy paint",        "#243A5E", 0.0, 0.50),  # 11
    ("Mustard paint",     "#D1A23A", 0.0, 0.55),  # 12
    ("Terracotta paint",  "#B5583B", 0.0, 0.60),  # 13
    ("Off-white paint",   "#EFEBE2", 0.0, 0.50),  # 14
    ("Charcoal paint",    "#3C4044", 0.0, 0.50),  # 15
    ("Black powder coat", "#1C1E20", 0.6, 0.45),  # 16
    ("White powder coat", "#E2E4E5", 0.2, 0.40),  # 17
    ("Raw steel",         "#8D9195", 0.9, 0.40),  # 18
    ("Brushed aluminium", "#C4C7CA", 1.0, 0.35),  # 19
    ("Brass",             "#C7A24A", 1.0, 0.30),  # 20
    ("Copper",            "#B86F50", 1.0, 0.35),  # 21
]
PAL = {}
TOP_CHOICES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
FRAME_CHOICES = [0, 1, 2, 3, 4, 9, 16, 16, 17, 18, 19, 10, 11, 14, 15]
ACCENT_CHOICES = [0, 1, 2, 3, 4, 5, 9, 10, 11, 12, 13, 14, 15, 16, 17]
METAL_CHOICES = [16, 18, 19, 20, 21]


def role_chain(role, idx_param, choices):
    prev = (PAL[choices[0]], "Value")
    for k in range(1, len(choices)):
        c = compare(idx_param, "Equal", k)
        s = N("SelectMaterial", f"{role} finish {k}", {})
        L(c, "Result", s, "Condition")
        L(prev[0], prev[1], s, "A")
        L(PAL[choices[k]], "Value", s, "B")
        prev = (s, "Result")
    return prev


def select_mat(cond_param, a, b, name):
    c = compare(cond_param, "Greater", 0.5)
    s = N("SelectMaterial", name, {})
    L(c, "Result", s, "Condition")
    L(a[0], a[1], s, "A")
    L(b[0], b[1], s, "B")
    return (s, "Result")


# =============================================================================
def build():
    # ---- exposed controls ---------------------------------------------------
    lit("seed", "Int32", 7, "Master random seed (0 to 1,000,000). Each seed gives a different table: type, size, finish and details.",
        group="Random", mn=0, mx=1000000)
    lit("table_type", "Int32", 0,
        "Pick a type, or leave on Random to let the seed decide.", group="Random",
        options=["Random (from seed)", "Four-leg table", "Trestle table", "Pedestal table", "Bench", "Bench with drawers"])
    lit("shuffle_type", "Int32", 0,
        "Change this number to re-roll only the table type and its structure (leg shape, aprons, shelves, drawer layout).",
        group="Random", mn=0, mx=1000)
    lit("shuffle_dimensions", "Int32", 0,
        "Change this number to re-roll only the dimensions (width, depth, height, thicknesses).", group="Random", mn=0, mx=1000)
    lit("shuffle_materials", "Int32", 0,
        "Change this number to re-roll only the materials and finishes.", group="Random", mn=0, mx=1000)
    lit("bevel", "Float", 0.006, "Edge bevel width in metres. Clamped on thin parts. 0 gives sharp edges.",
        group="Edges", mn=0.0, mx=0.03)
    lit("bevel_variation", "Float", 0.5,
        "How much the seed varies the bevel width: 0 is always exactly 'bevel', 1 ranges from none to double.",
        group="Edges", mn=0.0, mx=1.0)
    lit("bevel_segments", "Int32", 3, "Segments per bevel: 1 is a flat chamfer, more is a rounder edge.",
        group="Edges", mn=1, mx=6)

    # ---- aspect seeds -------------------------------------------------------
    ex("ts", "seed + shuffle_type*7919")
    ex("ds", "seed + shuffle_dimensions*7919")
    ex("ms", "seed + shuffle_materials*7919")

    # ---- type (structure) ---------------------------------------------------
    r_type = rnd("type", "ts")
    ex("type_i", f"min(1,table_type)*(table_type-1) + (1-min(1,table_type))*floor({r_type}*5)")
    for t in range(5):
        ex(f"is{t}", f"max(0, 1 - abs(type_i - {t}))")
    ex("legs_on", "is0 + is3")

    r = {}
    for n in ["legshape", "apron", "shelf", "rails", "lowy", "foot", "pedround", "colsq", "basesq", "full",
              "side", "nc", "nr", "hvert", "htop"]:
        r[n] = rnd(n, "ts")

    # ---- shared dimensions --------------------------------------------------
    for n in ["w", "d", "h", "topt", "bev"]:
        r[n] = rnd(n, "ds")
    lerp_t("W", r["w"], [(0.90, 2.00), (1.40, 2.40), (0.70, 1.30), (1.20, 2.20), (1.00, 2.00)])
    lerp_t("D_raw", r["d"], [(0.55, 1.00), (0.65, 0.95), (0.70, 1.30), (0.55, 0.80), (0.50, 0.75)])
    ex("D", "is2*W + (1-is2)*D_raw")
    lerp_t("H", r["h"], [(0.70, 0.78), (0.72, 0.78), (0.70, 0.76), (0.80, 0.95), (0.80, 0.92)])
    lerp_t("top_t", r["topt"], [(0.020, 0.050), (0.030, 0.060), (0.025, 0.050), (0.040, 0.090), (0.030, 0.060)])
    ex("top_y", "H - top_t/2")
    ex("leg_h", "max(0.1, H - top_t)")
    ex("leg_y", "leg_h/2")
    ex("top_r", "W/2")
    flag("ped_round", r["pedround"], 0.65)
    ex("top_round", "is2*ped_round")
    ex("bevel_eff", f"max(0, bevel*(1 + ({r['bev']}*2 - 1)*bevel_variation))")

    # ---- four-leg / bench ---------------------------------------------------
    for n in ["legw", "inset", "apronh", "apront", "railh", "railt", "shelft"]:
        r[n] = rnd(n, "ds")
    lerp_t("leg_w", r["legw"], [(0.035, 0.075), (0.035, 0.075), (0.035, 0.075), (0.070, 0.120), (0.035, 0.075)])
    ex("leg_r", "leg_w/2")
    lerp_t("inset", r["inset"], [(0.02, 0.12), (0.02, 0.12), (0.02, 0.12), (0.02, 0.08), (0.02, 0.12)])
    ex("lx", "max(0.05, W/2 - inset - leg_w/2)")
    ex("lz", "max(0.05, D/2 - inset - leg_w/2)")
    ex("nlx", "-lx")
    ex("nlz", "-lz")
    ex("len_x", "max(0.05, 2*lx - leg_w + 0.01)")
    ex("len_z", "max(0.05, 2*lz - leg_w + 0.01)")
    flag("leg_round", r["legshape"], [0.45, 0, 0, 0.15, 0])
    flag("apron_on", r["apron"], [0.60, 0, 0, 0.50, 0])
    lerp1("apron_h", r["apronh"], 0.06, 0.11)
    lerp1("apron_t", r["apront"], 0.018, 0.030)
    ex("apron_y", "H - top_t - apron_h/2")
    flag("shelf_on", r["shelf"], [0.20, 0, 0, 0.70, 0])
    flag("rails_raw", r["rails"], [0.35, 0, 0, 0.30, 0])
    ex("rails_on", "rails_raw*(1-shelf_on)")
    ex("low_y", f"leg_h*(0.12 + 0.16*{r['lowy']})")
    lerp1("rail_h", r["railh"], 0.03, 0.05)
    lerp1("rail_t", r["railt"], 0.02, 0.03)
    lerp1("shelf_t", r["shelft"], 0.02, 0.035)

    # ---- trestle ------------------------------------------------------------
    for n in ["endt", "endinset", "enddz", "beamh", "beamt", "drop", "footh"]:
        r[n] = rnd(n, "ds")
    lerp1("end_t", r["endt"], 0.03, 0.06)
    lerp1("end_inset", r["endinset"], 0.06, 0.25)
    ex("end_x", "max(0.1, W/2 - end_inset - end_t/2)")
    ex("nend_x", "-end_x")
    lerp1("end_dz", r["enddz"], 0.04, 0.14)
    ex("end_d", "max(0.2, D - 2*end_dz)")
    lerp1("beam_h", r["beamh"], 0.06, 0.14)
    lerp1("beam_t", r["beamt"], 0.04, 0.08)
    ex("beam_len", "max(0.1, 2*end_x - end_t + 0.01)")
    ex("beam_y", f"leg_h - beam_h/2 - {r['drop']}*0.3*leg_h")
    ex("foot_on", f"floor({r['foot']} + 0.55*is1)")
    lerp1("foot_h", r["footh"], 0.04, 0.07)
    ex("foot_w", "end_t + 0.06")
    ex("foot_d", "min(D - 0.02, end_d + 0.12)")
    ex("foot_y", "foot_h/2")

    # ---- pedestal -----------------------------------------------------------
    for n in ["colr", "baset", "baser"]:
        r[n] = rnd(n, "ds")
    lerp1("col_r", r["colr"], 0.035, 0.085)
    ex("col_w", "2*col_r")
    flag("col_sq", r["colsq"], 0.35)
    lerp1("base_t", r["baset"], 0.03, 0.07)
    ex("base_r", f"min(W/2 - 0.02, W*(0.28 + {r['baser']}*0.14))")
    ex("base_w", "2*base_r")
    flag("base_sq", r["basesq"], 0.30)
    ex("col_h", "max(0.1, H - top_t - base_t)")
    ex("col_y", "base_t + col_h/2")
    ex("base_y", "base_t/2")

    # ---- bench with drawers -------------------------------------------------
    for n in ["ovx", "ovz", "frac", "plinth", "plin", "ept", "ft", "hl", "ht", "hd"]:
        r[n] = rnd(n, "ds")
    lerp1("ov_x", r["ovx"], 0.0, 0.05)
    lerp1("ov_z", r["ovz"], 0.0, 0.04)
    ex("full", f"floor({r['full']} + 0.55*is4)")
    ex("side", f"2*floor({r['side']}+0.5)-1")
    lerp1("frac", r["frac"], 0.38, 0.65)
    lerp1("ep_t", r["ept"], 0.03, 0.05)
    # pedestal-style cabinet: never leave less than 30 cm of open knee space beside it
    ex("Wc", "full*(W - 2*ov_x) + (1-full)*min(W*frac, W - 2*ov_x - ep_t - 0.3)")
    ex("xc", "(1-full)*side*(W/2 - ov_x - Wc/2)")
    ex("Dc", "max(0.2, D - 2*ov_z)")
    lerp1("plinth_h", r["plinth"], 0.04, 0.10)
    lerp1("plin_in", r["plin"], 0.015, 0.04)
    ex("ch", "max(0.2, H - top_t - plinth_h)")
    ex("cab_y", "plinth_h + ch/2")
    ex("plinth_y", "plinth_h/2")
    ex("plinth_w", "max(0.1, Wc - 2*plin_in)")
    ex("plinth_d", "max(0.1, Dc - 2*plin_in)")
    ex("has_ep", "1-full")
    ex("ep_x", "-side*(W/2 - ov_x - ep_t/2)")
    ex("nc", f"1 + floor({r['nc']}*min(3, max(1, floor(Wc/0.3))))")
    ex("nr", f"1 + floor({r['nr']}*min(4, max(1, floor(ch/0.16))))")
    ex("cw", "(Wc - 0.024)/nc")
    ex("chh", "(ch - 0.024)/nr")
    ex("fw", "max(0.05, cw - 0.006)")
    ex("fh", "max(0.05, chh - 0.006)")
    lerp1("ft", r["ft"], 0.016, 0.026)
    ex("fz", "Dc/2 + ft/2 - 0.003")
    for i in range(3):
        ex(f"dx{i}", f"xc - Wc/2 + 0.012 + ({i}+0.5)*cw")
        ex(f"ac{i}", f"max(0, min(1, nc - {i}))")
    for j in range(4):
        ex(f"dy{j}", f"plinth_h + 0.012 + ({j}+0.5)*chh")
        ex(f"ar{j}", f"max(0, min(1, nr - {j}))")
    for i in range(3):
        for j in range(4):
            ex(f"act_{i}_{j}", f"ac{i}*ar{j}")
    lerp1("hl", r["hl"], 0.08, 0.18)
    lerp1("ht", r["ht"], 0.010, 0.016)
    lerp1("hd", r["hd"], 0.018, 0.030)
    flag("hvert", r["hvert"], 0.3)
    flag("htop", r["htop"], 0.4)
    ex("hl_eff", "min(hl, min(0.5*fw, 0.6*fh))")
    ex("hx", "hl_eff*(1-hvert) + ht*hvert")
    ex("hy", "ht*(1-hvert) + hl_eff*hvert")
    ex("hz", "ft/2 + hd/2 - 0.002")
    ex("hoff", "htop*(1-hvert)*(fh/2 - 0.04)")

    # ---- materials ----------------------------------------------------------
    for n in ["mtop", "mframe", "msame", "macc", "msameacc", "mmetal"]:
        r[n] = rnd(n, "ms")
    ex("idx_top", f"floor({r['mtop']}*{len(TOP_CHOICES)})")
    ex("idx_frame", f"floor({r['mframe']}*{len(FRAME_CHOICES)})")
    flag("same_frame", r["msame"], 0.25)
    ex("idx_acc", f"floor({r['macc']}*{len(ACCENT_CHOICES)})")
    flag("same_acc", r["msameacc"], 0.30)
    ex("idx_metal", f"floor({r['mmetal']}*{len(METAL_CHOICES)})")

    # ======================= nodes ===========================================
    SECTION[0] = "Finishes"
    for i, (nm, hx, me, ro) in enumerate(PALETTE):
        PAL[i] = N("ValueMaterial", f"{nm}", {"Value": MAT(hexlin(hx, me, ro))})
    top_m = role_chain("Top", "idx_top", TOP_CHOICES)
    frame_pick = role_chain("Frame", "idx_frame", FRAME_CHOICES)
    frame_m = select_mat("same_frame", frame_pick, top_m, "Frame matches top")
    acc_pick = role_chain("Accent", "idx_acc", ACCENT_CHOICES)
    acc_m = select_mat("same_acc", acc_pick, frame_m, "Accent matches frame")
    metal_m = role_chain("Hardware", "idx_metal", METAL_CHOICES)

    # ---- top ----------------------------------------------------------------
    SECTION[0] = "Top"
    top_rect = part("Table top", "cube", ["W", "top_t", "D"], top_m, pos=(0, "top_y", 0))
    top_rnd = part("Table top round", "cyl", ["top_r", "top_t"], top_m, sides=64, pos=(0, "top_y", 0))
    top_sel = variant("top_round", top_rect, top_rnd, "Round top?")

    # ---- four-leg / bench ---------------------------------------------------
    SECTION[0] = "Four-leg table and bench"
    leg_sq = part("Leg", "cube", ["leg_w", "leg_h", "leg_w"], frame_m)
    leg_rd = part("Leg round", "cyl", ["leg_r", "leg_h"], frame_m, sides=24)
    leg_any = variant("leg_round", leg_sq, leg_rd, "Round legs?")
    legs = [place(leg_any, (sx, "leg_y", sz), f"Leg {nm}")
            for nm, sx, sz in [("front left", "nlx", "lz"), ("front right", "lx", "lz"),
                               ("back left", "nlx", "nlz"), ("back right", "lx", "nlz")]]

    def rail_frame(label, y, h, t, mat):
        fb = part(f"{label} rail long", "cube", ["len_x", h, t], mat)
        lr = part(f"{label} rail short", "cube", [t, h, "len_z"], mat)
        return [place(fb, (0, y, "lz"), f"{label} rail front"), place(fb, (0, y, "nlz"), f"{label} rail back"),
                place(lr, ("nlx", y, 0), f"{label} rail left"), place(lr, ("lx", y, 0), f"{label} rail right")]

    apron = join_all(rail_frame("Apron", "apron_y", "apron_h", "apron_t", frame_m), "Apron")
    apron_g = gate(apron, "apron_on", "Show apron")
    low_rails = join_all(rail_frame("Lower", "low_y", "rail_h", "rail_t", frame_m), "Lower rails")
    low_rails_g = gate(low_rails, "rails_on", "Show lower rails")
    shelf = part("Lower shelf", "cube", ["len_x", "shelf_t", "len_z"], acc_m, pos=(0, "low_y", 0))
    shelf_g = gate(shelf, "shelf_on", "Show lower shelf")
    legs_asm = join_all(legs + [apron_g, low_rails_g, shelf_g], "Four-leg parts")
    legs_g = gate(legs_asm, "legs_on", "Four-leg or bench")

    # ---- trestle ------------------------------------------------------------
    SECTION[0] = "Trestle table"
    end_m = part("Trestle end", "cube", ["end_t", "leg_h", "end_d"], frame_m)
    ends = [place(end_m, ("end_x", "leg_y", 0), "Trestle end right"),
            place(end_m, ("nend_x", "leg_y", 0), "Trestle end left")]
    beam = part("Trestle beam", "cube", ["beam_len", "beam_h", "beam_t"], acc_m, pos=(0, "beam_y", 0))
    foot_m = part("Trestle foot", "cube", ["foot_w", "foot_h", "foot_d"], frame_m)
    feet = join_all([place(foot_m, ("end_x", "foot_y", 0), "Foot right"),
                     place(foot_m, ("nend_x", "foot_y", 0), "Foot left")], "Feet")
    feet_g = gate(feet, "foot_on", "Show feet")
    tres_asm = join_all(ends + [beam, feet_g], "Trestle parts")
    tres_g = gate(tres_asm, "is1", "Trestle")

    # ---- pedestal -----------------------------------------------------------
    SECTION[0] = "Pedestal table"
    col_rd = part("Column round", "cyl", ["col_r", "col_h"], frame_m, sides=32)
    col_sq = part("Column square", "cube", ["col_w", "col_h", "col_w"], frame_m)
    col = place(variant("col_sq", col_rd, col_sq, "Square column?"), (0, "col_y", 0), "Column position")
    base_rd = part("Base round", "cyl", ["base_r", "base_t"], frame_m, sides=48)
    base_sq = part("Base square", "cube", ["base_w", "base_t", "base_w"], frame_m)
    base = place(variant("base_sq", base_rd, base_sq, "Square base?"), (0, "base_y", 0), "Base position")
    ped_asm = join_all([col, base], "Pedestal parts")
    ped_g = gate(ped_asm, "is2", "Pedestal")

    # ---- bench with drawers -------------------------------------------------
    SECTION[0] = "Bench with drawers"
    body = part("Cabinet body", "cube", ["Wc", "ch", "Dc"], frame_m, pos=("xc", "cab_y", 0))
    plinth = part("Cabinet plinth", "cube", ["plinth_w", "plinth_h", "plinth_d"], frame_m, pos=("xc", "plinth_y", 0))
    endp = part("Cabinet end panel", "cube", ["ep_t", "leg_h", "Dc"], frame_m, pos=("ep_x", "leg_y", 0))
    endp_g = gate(endp, "has_ep", "Show end panel")
    front = part("Drawer front", "cube", ["fw", "fh", "ft"], acc_m)
    handle = part("Drawer handle", "cube", ["hx", "hy", "hd"], metal_m, pos=(0, "hoff", "hz"))
    unit = join_all([front, handle], "Drawer")
    slots = []
    for i in range(3):
        for j in range(4):
            p = place(unit, (f"dx{i}", f"dy{j}", "fz"), f"Drawer {i + 1}.{j + 1}")
            slots.append(gate(p, f"act_{i}_{j}", f"Show drawer {i + 1}.{j + 1}"))
    drawers = join_all(slots, "Drawers")
    drw_asm = join_all([body, plinth, endp_g, drawers], "Drawer bench parts")
    drw_g = gate(drw_asm, "is4", "Bench with drawers")

    # ---- final --------------------------------------------------------------
    SECTION[0] = "Assembly"
    out = join_all([top_sel, legs_g, tres_g, ped_g, drw_g], "Assemble table")
    return out


# ---- emit -------------------------------------------------------------------
def val_text(spec):
    k, x = spec
    if k == "param": return f"{{param: {x}}}"
    if k == "Float": return f"{{kind: Float, value: {fmt(x)}}}"
    if k == "Int32": return f"{{kind: Int32, value: {x}}}"
    if k == "String": return f"{{kind: String, value: \"{x}\"}}"
    if k == "Menu": return f"{{kind: Menu, value: {x}}}"
    if k == "Float3": return f"{{kind: Float3, value: [{', '.join(fmt(c) for c in x)}]}}"
    if k == "Material": return f"{{kind: Material, value: [{', '.join(fmt(c) for c in x)}]}}"
    raise ValueError(k)


def layout():
    depth = {}
    incoming = defaultdict(list)
    for a, ap, b, bp in LINKS:
        incoming[b].append(a)
    by_id = {n["id"]: n for n in NODES}

    def d(nid):
        if nid in depth:
            return depth[nid]
        srcs = incoming.get(nid, [])
        depth[nid] = 0 if not srcs else 1 + max(d(s) for s in srcs)
        return depth[nid]

    for n in NODES:
        d(n["id"])
    order = SECTION_ORDER
    y_base = {}
    cursor = 40
    pos = {}
    for sec in order:
        rows = defaultdict(int)
        for n in NODES:
            if n["section"] != sec:
                continue
            dep = depth[n["id"]]
            pos[n["id"]] = (40 + dep * 300, cursor + rows[dep] * 130)
            rows[dep] += 1
        cursor += (max(rows.values()) if rows else 1) * 130 + 160
    return pos


def emit(path, out_node):
    pos = layout()
    lines = []
    lines.append("# Procedural table. Change `seed` for a different table (type, size, finish, details).")
    lines.append("# table_type forces a type; shuffle_type / shuffle_dimensions / shuffle_materials re-roll one aspect only.")
    lines.append("# bevel / bevel_variation / bevel_segments control the rounded edges on every part.")
    lines.append("# All randomness is derived from `seed` by the derived (expr) parameters at the end of this file.")
    lines.append("nodes:")
    for n in NODES:
        inp = ", ".join(f"{k}: {val_text(s)}" for k, s in n["inputs"].items())
        x, y = pos[n["id"]]
        name = n["name"].replace('"', "'")
        parts = [f"id: {n['id']}", f"type: {n['type']}", f'name: "{name}"']
        if inp:
            parts.append(f"inputs: {{{inp}}}")
        parts += [f"x: {x}", f"y: {y}"]
        lines.append("  - {" + ", ".join(parts) + "}")
    lines.append("links:")
    for a, ap, b, bp in LINKS:
        lines.append(f"  - {{from: {{node: {a}, pin: {ap}}}, to: {{node: {b}, pin: {bp}}}}}")
    lines.append("parameters:")
    for name, p in PARAMS.items():
        items = []
        for k, x in p.items():
            if k == "options":
                items.append("options: [" + ", ".join(f'"{o}"' for o in x) + "]")
            elif k in ("desc", "group", "expr"):
                items.append(f'{k}: "{x}"')
            elif isinstance(x, float):
                items.append(f"{k}: {fmt(x)}")
            else:
                items.append(f"{k}: {x}")
        lines.append(f"  {name}: {{{', '.join(items)}}}")
    lines.append("outputs:")
    lines.append("  Geometry:")
    lines.append(f"    node: {out_node}")
    lines.append("    pin: Geometry")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    out = build()
    emit(sys.argv[1], out)
    print(f"nodes={len(NODES)} links={len(LINKS)} params={len(PARAMS)} out={out}")
