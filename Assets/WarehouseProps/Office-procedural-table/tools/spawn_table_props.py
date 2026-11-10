#!/usr/bin/env python3
"""Spawn copies of the ScatterProps_Phys props on the procedural table, using the layout the
procedural-table-props pod graph defines (its derived params are evaluated here with the same maths).

usage: spawn_table_props.py SCENE_IN SCENE_OUT [--count N] [--prop-seed S] [--tumble T] [--drop H]
                            [--margin M] [--table-tag procedural-table]
Writes a NEW scene file; the input scene is never modified.
"""
import argparse, math, random, re, sys, secrets
sys.path.insert(0, __import__("os").path.dirname(__file__))
import gen_props, gen_table as gt

ap = argparse.ArgumentParser()
ap.add_argument("scene_in"); ap.add_argument("scene_out")
ap.add_argument("--count", type=int, default=6)
ap.add_argument("--prop-seed", type=int, default=1)
ap.add_argument("--tumble", type=float, default=0.0)
ap.add_argument("--drop", type=float, default=0.12)
ap.add_argument("--margin", type=float, default=0.07)
ap.add_argument("--table-tag", default="procedural-table")
ap.add_argument("--parent-tag", default="ScatterProps_Phys")
ap.add_argument("--move", action="store_true", help="move the ScatterProps_Phys originals instead of spawning copies")
ap.add_argument("--json", action="store_true", help="print placements only")
a = ap.parse_args()

text = open(a.scene_in, encoding="utf-8").read()
head, rest = text.split("\nEntities:\n", 1)
body, tail = rest.split("\nSceneAudio:", 1)
blocks = re.split(r"(?m)^  - Entity: ", body)
pre, blocks = blocks[0], blocks[1:]          # pre is empty
ents = {}
for b in blocks:
    eid = b.split("\n", 1)[0].strip()
    ents[eid] = b


def tag(b): return re.search(r"Tag: (.*)", b).group(1).strip()


table = next(b for b in ents.values() if tag(b) == a.table_tag and "PodComponent:" in b)
pos = [float(x) for x in re.search(r"Position: \[(.*?)\]", table).group(1).split(",")]
rot = [float(x) for x in re.search(r"Rotation: \[(.*?)\]", table).group(1).split(",")]
overrides = {int(m.group(1)): float(m.group(2)) for m in re.finditer(r"- Node: (\d+)\n\s+Float: \w+\n\s+V: ([-\d.eE+]+)", table)}
# editor numbers literal params after the highest node id: seed=355, table_type=356, shuffle_type=357, shuffle_dimensions=358
vals = {"seed": overrides.get(355, 7), "table_type": overrides.get(356, 0),
        "shuffle_type": overrides.get(357, 0), "shuffle_dimensions": overrides.get(358, 0)}
if any(abs(r) > 1e-6 for r in rot):
    sys.exit("table has a rotation; this script only handles an unrotated table")

parent = next(b for b in ents.values() if tag(b) == a.parent_tag)
child_ids = re.findall(r"- Handle: (\d+)", parent.split("TransformComponent:")[0])

# ---- evaluate the graph's derived params (same exprs the pod graph runs) ----
gen_props.build()
env = {"min": min, "max": max, "abs": abs, "floor": math.floor, "ceil": math.ceil, "round": round,
       "sin": math.sin, "cos": math.cos, "tan": math.tan, "pow": pow, "sqrt": math.sqrt, "pi": math.pi}
vals.update({"prop_seed": a.prop_seed, "prop_count": a.count, "edge_margin": a.margin,
             "drop_height": a.drop, "tumble": a.tumble})
for name, p in gt.PARAMS.items():
    if "expr" in p:
        env[name] = vals[name] = float(eval(p["expr"], {"__builtins__": {}}, {**env, **vals}))
    else:
        vals.setdefault(name, p["default"])
n = a.count
place = []
for i in range(n):
    place.append(dict(slot=i, prop=int(vals[f"prop{i}"]), x=vals[f"x{i}"], y=vals["surf_y"], z=vals[f"z{i}"],
                      yaw=vals[f"yaw{i}"], tx=vals[f"tilt_x{i}"], tz=vals[f"tilt_z{i}"]))
if a.json:
    import json; print(json.dumps(dict(table_pos=pos, top=dict(W=vals["W"], D=vals["D"], H=vals["H"], round=vals["top_round"]),
                                       placements=place), indent=1)); sys.exit()


# ---- quaternion helpers (glm conventions: q = qz*qy*qx for euler (x,y,z)) ----
def qmul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)
def qaxis(ax, ang):
    s = math.sin(ang/2); return (math.cos(ang/2), ax[0]*s, ax[1]*s, ax[2]*s)
def q_from_euler(e): return qmul(qaxis((0,0,1), e[2]), qmul(qaxis((0,1,0), e[1]), qaxis((1,0,0), e[0])))
def euler_from_q(q):
    w, x, y, z = q
    roll = math.atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z)
    pitch = math.atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z)
    yaw = math.asin(max(-1.0, min(1.0, -2*(x*z - w*y))))
    return [pitch, yaw, roll]


def fmt(v): return f"{v:.6g}"
def new_id(used):
    while True:
        i = str(secrets.randbits(63) | (1 << 62))
        if i not in used: used.add(i); return i


if a.move:
    assert a.count <= len(child_ids)
    # footprint radii (m) per prop, from the mesh sizes in StanfordProps/*.gltf (conservative, orientation unknown)
    RADII = {"baseball": 0.05, "elephant": 0.065, "lubricant": 0.085, "cat_statue": 0.15, "apple": 0.055,
             "gnome": 0.13, "waterbottle": 0.09, "bucket": 0.13, "tongue": 0.13, "pliers": 0.1}
    def radius(i):
        tg = tag(ents[child_ids[i % len(child_ids)]])
        return next((r for k, r in RADII.items() if k in tg), 0.1)
    rail_t = 0.02
    lim_x = vals["W"] / 2 - rail_t
    lim_z = vals["D"] / 2 - rail_t
    if vals["top_round"] > 0.5:
        lim_x = lim_z = vals["top_r"] * 0.9 - rail_t
    for it in range(600):
        moved = 0.0
        for p in place:
            p["r"] = radius(p["prop"])
        for ai in range(len(place)):
            for bi in range(ai + 1, len(place)):
                A, B = place[ai], place[bi]
                dx, dz = B["x"] - A["x"], B["z"] - A["z"]
                d = math.hypot(dx, dz) or 1e-6
                gap = A["r"] + B["r"] + 0.01 - d
                if gap > 0:
                    ux, uz = dx / d, dz / d
                    A["x"] -= ux * gap / 2; A["z"] -= uz * gap / 2
                    B["x"] += ux * gap / 2; B["z"] += uz * gap / 2
                    moved += gap
        for p in place:
            r = min(p["r"], 0.9 * min(lim_x, lim_z))
            p["x"] = max(-lim_x + r, min(lim_x - r, p["x"]))
            p["z"] = max(-lim_z + r, min(lim_z - r, p["z"]))
        if moved < 1e-5:
            break
    print(f"  relaxed overlaps in {it + 1} iterations (residual {moved:.4f})")
    for pl in place:
        src_id = child_ids[pl["prop"] % len(child_ids)]
        b = ents[src_id]
        base = [float(x) for x in re.search(r"Rotation: \[(.*?)\]", b).group(1).split(",")]
        q = qmul(qaxis((0, 1, 0), pl["yaw"]), qmul(qmul(qaxis((0, 0, 1), pl["tz"]), qaxis((1, 0, 0), pl["tx"])), q_from_euler(base)))
        e = euler_from_q(q)
        wp = [pos[0] + pl["x"], pos[1] + pl["y"], pos[2] + pl["z"]]
        b = re.sub(r"(?m)^(      Position: )\[.*\]", lambda m: m.group(1) + "[" + ", ".join(fmt(v) for v in wp) + "]", b, count=1)
        b = re.sub(r"(?m)^(      Rotation: )\[.*\]", lambda m: m.group(1) + "[" + ", ".join(fmt(v) for v in e) + "]", b, count=1)
        ents[src_id] = b
        print(f"  moved {tag(b)}: {wp[0]:.3f} {wp[1]:.3f} {wp[2]:.3f} yaw={math.degrees(pl['yaw']):.0f}")
    out = head + "\nEntities:\n" + "".join("  - Entity: " + (b if b.endswith("\n") else b + "\n") for b in ents.values()) + "SceneAudio:" + tail
    open(a.scene_out, "w", encoding="utf-8", newline="\n").write(out)
    print("wrote", a.scene_out)
    sys.exit()

used = set(ents)
parent_id = new_id(used)
new_blocks, new_children = [], []
for pl in place:
    src_id = child_ids[pl["prop"] % len(child_ids)]
    b = ents[src_id]
    eid = new_id(used)
    base = [float(x) for x in re.search(r"Rotation: \[(.*?)\]", b).group(1).split(",")]
    q = qmul(qaxis((0,1,0), pl["yaw"]), qmul(qmul(qaxis((0,0,1), pl["tz"]), qaxis((1,0,0), pl["tx"])), q_from_euler(base)))
    e = euler_from_q(q)
    wp = [pos[0] + pl["x"], pos[1] + pl["y"], pos[2] + pl["z"]]
    b2 = re.sub(r"^(\S+)", eid, b, count=1)
    b2 = re.sub(r"(?m)^(    Parent: )\d+", rf"\g<1>{parent_id}", b2, count=1)
    b2 = re.sub(r"(?m)^(      Position: )\[.*\]", rf"\g<1>[{', '.join(fmt(v) for v in wp)}]", b2, count=1)
    b2 = re.sub(r"(?m)^(      Rotation: )\[.*\]", rf"\g<1>[{', '.join(fmt(v) for v in e)}]", b2, count=1)
    new_blocks.append(b2.rstrip("\n"))
    new_children.append(eid)

parent_block = (f"{parent_id}\n    Domain: Client\n    TagComponent:\n      Tag: TableProps_Phys\n    Parent: 0\n"
                "    Children:\n" + "".join(f"      - Handle: {c}\n" for c in new_children) +
                "    TransformComponent:\n      Position: [0, 0, 0]\n      Rotation: [0, 0, 0]\n      Scale: [1, 1, 1]")
out = head + "\nEntities:\n" + pre + "".join("  - Entity: " + b + "\n" if not b.endswith("\n") else "  - Entity: " + b for b in blocks)
out = out.rstrip("\n") + "\n  - Entity: " + parent_block + "\n" + "".join("  - Entity: " + b + "\n" for b in new_blocks)
out += "SceneAudio:" + tail
open(a.scene_out, "w", encoding="utf-8", newline="\n").write(out)
print(f"wrote {a.scene_out}: {len(place)} props on table seed {int(vals['seed'])} "
      f"(top {vals['W']:.2f} x {vals['D']:.2f}, round={int(vals['top_round'])}) at {pos}")
for pl in place:
    print(f"  prop {pl['prop']} ({tag(ents[child_ids[pl['prop'] % len(child_ids)]])}): x={pl['x']:.3f} z={pl['z']:.3f} yaw={math.degrees(pl['yaw']):.0f} deg")
