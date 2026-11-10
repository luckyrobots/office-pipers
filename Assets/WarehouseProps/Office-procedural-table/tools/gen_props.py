#!/usr/bin/env python3
"""procedural-table-props.hpodgraph: seeded prop placements on the procedural table's top.

Re-derives the table's top size from the same seed/shuffle params (identical hash constants), then lays
out up to 10 props in jittered grid cells (no overlap by construction), each with a random yaw (and optional
tumble). Markers show each placement; `placements.py` evaluates the same exprs to spawn the real props.
"""
import sys
import gen_table as gt
from gen_table import *  # noqa

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
    lit("prop_seed", "Int32", 1, "Re-rolls where the props go and how they are turned.", group="Props", mn=0, mx=1000000)
    lit("prop_count", "Int32", 6, "How many props to place (1 to 10).", group="Props", mn=1, mx=NPROPS)
    lit("edge_margin", "Float", 0.07, "Keep props this far from the table edge (metres).", group="Props", mn=0.0, mx=0.3)
    lit("drop_height", "Float", 0.12, "Height above the table top where props start; physics drops them.",
        group="Props", mn=0.0, mx=0.5)
    lit("tumble", "Float", 0.0, "0 keeps props upright with only a random turn; 1 gives fully random orientations.",
        group="Props", mn=0.0, mx=1.0)

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

    # ---- placement area: rect, or the square inscribed in a round top
    ex("hx", "max(0.02, (1-top_round)*(W/2-edge_margin) + top_round*(top_r*0.7071-edge_margin))")
    ex("hz", "max(0.02, (1-top_round)*(D/2-edge_margin) + top_round*(top_r*0.7071-edge_margin))")
    ex("cols", "min(prop_count, max(1, floor(sqrt(prop_count*hx/hz) + 0.5)))")
    ex("rows", "ceil(prop_count/cols)")
    ex("cells", "cols*rows")
    ex("cw", "2*hx/cols")
    ex("ch_", "2*hz/rows")
    ps = "prop_seed"
    r_off = rnd("celloff", ps)
    r_prop = rnd("propbase", ps)
    ex("cell_off", f"floor({r_off}*cells)")
    ex("prop_base", f"floor({r_prop}*{NPROPS})")
    ex("surf_y", "H + drop_height")

    slots = []
    for i in range(NPROPS):
        act = ex(f"act{i}", f"min(1, max(0, prop_count - {i}))")
        ex(f"cell{i}", f"({i}+cell_off) - cells*floor(({i}+cell_off)/cells)")
        ex(f"col{i}", f"cell{i} - cols*floor(cell{i}/cols)")
        ex(f"row{i}", f"floor(cell{i}/cols)")
        jx, jz, ya, rx_, rz_ = (rnd(f"s{i}_{n}", ps) for n in ["jx", "jz", "yaw", "rx", "rz"])
        ex(f"x{i}", f"-hx + (col{i}+0.5)*cw + ({jx}-0.5)*0.7*cw")
        ex(f"z{i}", f"-hz + (row{i}+0.5)*ch_ + ({jz}-0.5)*0.7*ch_")
        ex(f"yaw{i}", f"{ya}*6.2832")
        ex(f"tilt_x{i}", f"tumble*({rx_}-0.5)*6.2832")
        ex(f"tilt_z{i}", f"tumble*({rz_}-0.5)*3.1416")
        ex(f"prop{i}", f"({i}+prop_base) - {NPROPS}*floor(({i}+prop_base)/{NPROPS})")
        slots.append(act)

    # ---- nodes: top slab for context + one marker per slot
    gt.SECTION[0] = "Table top"
    top_rect = N("MeshCube", "Table top", {"SizeX": P("W"), "SizeY": P("top_t"), "SizeZ": P("D")})
    top_rect_t = place(top_rect, (0, "top_y", 0), "Table top position")
    top_rnd = N("MeshCylinder", "Table top round", {"Sides": I(64), "Radius": P("top_r"), "Height": P("top_t")})
    top_rnd_t = place(top_rnd, (0, "top_y", 0), "Table top round position")
    top = variant("top_round", top_rect_t, top_rnd_t, "Round top?")

    gt.SECTION[0] = "Prop placements"
    marks = []
    for i in range(NPROPS):
        cube = N("MeshCube", f"Prop {i + 1} marker", {"SizeX": F(0.14), "SizeY": F(0.02), "SizeZ": F(0.06)})
        pos = N("CombineXYZ", f"Prop {i + 1} position", {"X": P(f"x{i}"), "Y": P("surf_y"), "Z": P(f"z{i}")})
        rot = N("CombineXYZ", f"Prop {i + 1} rotation", {"X": P(f"tilt_x{i}"), "Y": P(f"yaw{i}"), "Z": P(f"tilt_z{i}")})
        t = N("Transform", f"Prop {i + 1} placement", {})
        L(cube, "Geometry", t, "Geometry")
        L(pos, "Vector", t, "Translation")
        L(rot, "Vector", t, "Rotation")
        marks.append(gate(t, f"act{i}", f"Show prop {i + 1}"))
    gt.SECTION[0] = "Assembly"
    import os
    return join_all(([] if os.environ.get("NOTOP") else [top]) + marks, "Props and top")


if __name__ == "__main__":
    out = build()
    emit(sys.argv[1], out)
    print(f"nodes={len(NODES)} links={len(LINKS)} params={len(PARAMS)} out={out}")
