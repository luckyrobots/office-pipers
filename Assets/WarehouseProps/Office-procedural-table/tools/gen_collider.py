#!/usr/bin/env python3
"""procedural-table-collider.hpodgraph: seeded prop placements on the procedural table's top.

Re-derives the table's top size from the same seed/shuffle params (identical hash constants), then lays
out up to 10 props in jittered grid cells (no overlap by construction), each with a random yaw (and optional
tumble). Markers show each placement; `placements.py` evaluates the same exprs to spawn the real props.
"""
import sys
import gen_table as gt
from gen_table import *  # noqa

import math
NPROPS = 10
PROP_NAMES = ["baseball", "elephant", "cat statue", "apple", "gnome", "lubricant spray", "pliers",
              "scene", "tongue-groove pliers", "waterbottle"]


def build():
    lit("seed", "Int32", 7, "Table seed. Use the SAME value as the procedural table so the top size matches.",
        group="Table match", mn=0, mx=1000000)
    lit("table_type", "Int32", 0, "Same as the table's table_type.", group="Table match",
        options=["Random (from seed)", "Four-leg table", "Trestle table", "Pedestal table", "Bench", "Bench with drawers"])
    lit("shuffle_type", "Int32", 0, "Same as the table's shuffle_type.", group="Table match", mn=0, mx=1000)
    lit("shuffle_dimensions", "Int32", 0, "Same as the table's shuffle_dimensions.", group="Table match", mn=0, mx=1000)
    lit("rail_height", "Float", 0.06, "Height of the collision rim above the table top (metres).", group="Collider", mn=0.01, mx=0.3)
    lit("rail_thickness", "Float", 0.03, "Thickness of the collision rim (metres).", group="Collider", mn=0.01, mx=0.1)
    lit("rail_gap", "Float", 0.012, "Gap between the table edge and the rim. Keep it above ~1 cm so physics treats the rim and the top as separate shapes (no ramp).", group="Collider", mn=0.011, mx=0.05)

    ex("ts", "seed + shuffle_type*7919")
    ex("ds", "seed + shuffle_dimensions*7919")
    # same rnd() call order as gen_table.build(), so the hash constants (and so the table top) match exactly
    r_type = rnd("type", "ts")
    ex("type_i", f"min(1,table_type)*(table_type-1) + (1-min(1,table_type))*floor({r_type}*5)")
    for t in range(5):
        ex(f"is{t}", f"max(0, 1 - abs(type_i - {t}))")
    names = ["legshape", "apron", "shelf", "rails", "lowy", "foot", "pedround", "colsq", "basesq", "full",
             "side", "nc", "nr", "hvert", "htop"]
    r = {}
    for n in names:
        if n == "pedround":
            r[n] = rnd(n, "ts")
        else:
            gt._k[0] += 1
    for n in ["w", "d", "h", "topt", "bev"]:
        r[n] = rnd(n, "ds")
    lerp_t("W", r["w"], [(0.90, 2.00), (1.40, 2.40), (0.70, 1.30), (1.20, 2.20), (1.00, 2.00)])
    lerp_t("D_raw", r["d"], [(0.55, 1.00), (0.65, 0.95), (0.70, 1.30), (0.55, 0.80), (0.50, 0.75)])
    ex("D", "is2*W + (1-is2)*D_raw")
    lerp_t("H", r["h"], [(0.70, 0.78), (0.72, 0.78), (0.70, 0.76), (0.80, 0.95), (0.80, 0.92)])
    lerp_t("top_t", r["topt"], [(0.020, 0.050), (0.030, 0.060), (0.025, 0.050), (0.040, 0.090), (0.030, 0.060)])
    ex("top_y", "H - top_t/2")
    ex("top_r", "W/2")
    flag("ped_round", r["pedround"], 0.65)
    ex("top_round", "is2*ped_round")

    ex("rect_on", "1 - top_round")

    SEG = 20
    ex("rail_full", "top_t + rail_height")
    ex("rail_y", "H - top_t + rail_full/2")
    ex("long_len", "W + 2*(rail_gap + rail_thickness)")
    ex("short_len", "D")
    ex("zrail", "D/2 + rail_gap + rail_thickness/2")
    ex("xrail", "W/2 + rail_gap + rail_thickness/2")
    ex("nzrail", "-zrail")
    ex("nxrail", "-xrail")
    ex("ring_r", "top_r + rail_gap + rail_thickness/2")
    ex("seg_len", f"2*ring_r*sin({math.pi / SEG:.6f})*1.08")
    gt.SECTION[0] = "Collision pieces"
    top_rect = N("MeshCube", "Collider top", {"SizeX": P("W"), "SizeY": P("top_t"), "SizeZ": P("D")})
    top_rect_t = place(top_rect, (0, "top_y", 0), "Collider top position")
    top_rnd = N("MeshCylinder", "Collider top round", {"Sides": I(64), "Radius": P("top_r"), "Height": P("top_t")})
    top_rnd_t = place(top_rnd, (0, "top_y", 0), "Collider top round position")
    top = variant("top_round", top_rect_t, top_rnd_t, "Round top?")

    long_c = N("MeshCube", "Rail long", {"SizeX": P("long_len"), "SizeY": P("rail_full"), "SizeZ": P("rail_thickness")})
    short_c = N("MeshCube", "Rail short", {"SizeX": P("rail_thickness"), "SizeY": P("rail_full"), "SizeZ": P("short_len")})
    rails = [place(long_c, (0, "rail_y", "zrail"), "Rail front"), place(long_c, (0, "rail_y", "nzrail"), "Rail back"),
             place(short_c, ("xrail", "rail_y", 0), "Rail right"), place(short_c, ("nxrail", "rail_y", 0), "Rail left")]
    rect_rim = gate(join_all(rails, "Rect rim"), "rect_on", "Rectangular rim")

    seg_c = N("MeshCube", "Ring segment", {"SizeX": P("seg_len"), "SizeY": P("rail_full"), "SizeZ": P("rail_thickness")})
    segs = []
    for k in range(SEG):
        ex(f"sx{k}", f"ring_r*cos({2 * math.pi * k / SEG:.6f})")
        ex(f"sz{k}", f"ring_r*sin({2 * math.pi * k / SEG:.6f})")
        ex(f"sa{k}", f"{math.pi / 2 - 2 * math.pi * k / SEG:.6f}")
        pos = N("CombineXYZ", f"Ring {k + 1} position", {"X": P(f"sx{k}"), "Y": P("rail_y"), "Z": P(f"sz{k}")})
        rot = N("CombineXYZ", f"Ring {k + 1} rotation", {"X": F(0), "Y": P(f"sa{k}"), "Z": F(0)})
        t = N("Transform", f"Ring {k + 1} segment", {})
        L(seg_c, "Geometry", t, "Geometry"); L(pos, "Vector", t, "Translation"); L(rot, "Vector", t, "Rotation")
        segs.append(t)
    round_rim = gate(join_all(segs, "Round rim"), "top_round", "Round rim")

    gt.SECTION[0] = "Assembly"
    allc = join_all([top, rect_rim, round_rim], "Collision shapes")
    role = N("SetMeshOutputRole", "Collision only", {"Role": M("collision")})
    L(allc, "Geometry", role, "Geometry")
    return role


if __name__ == "__main__":
    out = build()
    emit(sys.argv[1], out)
    print(f"nodes={len(NODES)} links={len(LINKS)} params={len(PARAMS)} out={out}")
