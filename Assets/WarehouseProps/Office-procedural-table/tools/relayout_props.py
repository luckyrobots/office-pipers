#!/usr/bin/env python3
"""Re-lay the ScatterProps_Phys props on the procedural table to match the table's CURRENT seed in the
running editor (edit mode), and keep the collider entity's parameters in step.

Run after changing the table seed / shuffles:   py relayout_props.py   (or  py relayout_props.py --watch  to do it automatically)
Needs the editor open with the scene loaded. Edits are live and unsaved: save the scene afterwards (Ctrl+S).
"""
import argparse, json, math, os, re, sys, urllib.request
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_props, gen_table as gt

SETTINGS = os.path.join(HERE, "props_settings.json")
DEFAULTS = {"count": 10, "prop_seed": 1, "tumble": 0.0, "spawn_height_offset": 0.02, "margin": 0.07,
            "exclude_tags": ["waterbottle"]}


def load_settings():
    """props_settings.json beside this script; edit it (the watcher picks changes up) or override on the command line."""
    out = dict(DEFAULTS)
    try:
        out.update(json.load(open(SETTINGS, encoding="utf-8")))
    except FileNotFoundError:
        json.dump(DEFAULTS, open(SETTINGS, "w", encoding="utf-8"), indent=2)
    except Exception as ex:
        print("props_settings.json unreadable, using defaults:", ex)
    return out


cfg = load_settings()
ap = argparse.ArgumentParser()
ap.add_argument("--count", type=int, default=int(cfg["count"]))
ap.add_argument("--prop-seed", type=int, default=int(cfg["prop_seed"]))
ap.add_argument("--tumble", type=float, default=float(cfg["tumble"]))
ap.add_argument("--drop", "--spawn-height-offset", dest="drop", type=float, default=float(cfg["spawn_height_offset"]),
                help="metres between the table top and the LOWEST point of each prop when it spawns")
ap.add_argument("--margin", type=float, default=float(cfg["margin"]))
ap.add_argument("--watch", action="store_true", help="keep running and re-lay the props whenever the table or props_settings.json changes")
ap.add_argument("--auto-seed", action="store_true", help="with --watch: pick a new random table seed every time Play stops, so the next Play has a fresh table")
a = ap.parse_args()

URL = "http://127.0.0.1:19331/mcp"
HDR = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}


def post(body, sid=None):
    h = dict(HDR)
    if sid:
        h["Mcp-Session-Id"] = sid
    with urllib.request.urlopen(urllib.request.Request(URL, data=json.dumps(body).encode(), headers=h), timeout=120) as r:
        return r.headers.get("Mcp-Session-Id"), r.read().decode()


sid, _ = post({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
    "protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "relayout", "version": "1"}}})
post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)


def call(name, args):
    _, txt = post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": name, "arguments": args}}, sid)
    c = json.loads(txt)["result"]["content"][0]["text"]
    try:
        return json.loads(c)
    except Exception:
        return c


def fresh_ctx(timeout=30):
    """The authoring context changes after every Play/Stop (and while the scene reloads), so ask again each time."""
    import time
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            c = call("authoring_state", {})["native"].get("context_id")
            if c:
                return c
        except Exception:
            pass
        time.sleep(0.5)
    return None


ctx = fresh_ctx(5 if a.watch else 30)
if not ctx:
    if a.watch:
        import time
        time.sleep(3)
        os.execv(sys.executable, [sys.executable, os.path.abspath(__file__)] + sys.argv[1:])
    sys.exit("The editor has no editable scene open yet.")


def scene(req):
    c = fresh_ctx()
    if not c:
        raise RuntimeError("editor has no editable scene")
    return call("scene_authoring", {"context_id": c, "request": req})["native"]


if a.watch:
    # Keep running: whenever the table's seed / shuffles / type change (edit mode only), re-run this script once.
    import subprocess, time
    child = [sys.executable, os.path.abspath(__file__)] + [x for x in sys.argv[1:] if x != "--watch"]
    last = None
    prev_mode = None
    import random
    print("Watching the procedural-table seed. Leave this window open; Ctrl+C to stop.")
    while True:
        try:
            mode_now = call("harness/get_scene_state_mode", {}).get("name")
            just_stopped = a.auto_seed and mode_now == "Edit" and prev_mode in ("Play", "Preparing")
            if mode_now != "Preparing":
                prev_mode = mode_now
            if mode_now == "Edit":
                txt = call("search_entities", {"namePattern": "procedural-table"})
                txt = txt.get("result", "") if isinstance(txt, dict) else txt
                tid = dict((n, u) for n, u in re.findall(r"(procedural-table(?:-collider)?) \(UUID: (\d+)\)", txt)).get("procedural-table")
                if tid:
                    snap = scene({"schema_version": 1, "action": "inspect", "entities": [tid]})
                    if just_stopped:
                        new_seed = random.randint(1, 100000)
                        r = scene({"schema_version": 1, "action": "edit", "expected": snap["stamp"],
                                   "preconditions": [{"entity_id": tid, "fingerprint": snap["entities"][0]["fingerprint"]}],
                                   "edits": [{"op": "set_parameter", "entity_id": tid, "name": "seed",
                                              "value": {"kind": "Int32", "value": new_seed}}]})
                        print(time.strftime("%H:%M:%S"), "Play stopped -> new seed", new_seed, r.get("code"), r.get("message", ""))
                        snap = scene({"schema_version": 1, "action": "inspect", "entities": [tid]})
                    sig = json.dumps(sorted((o["node_id"], o["value"]["value"]) for o in snap["entities"][0]["pod"]["parameter_overrides"]))
                    sig += str(os.path.getmtime(SETTINGS) if os.path.exists(SETTINGS) else 0)
                    if sig != last:
                        print(time.strftime("%H:%M:%S"), "table changed ->", sig)
                        subprocess.run(child, check=False)
                        last = sig
        except KeyboardInterrupt:
            sys.exit(0)
        except Exception as ex:
            print("editor not reachable, retrying:", ex)
            time.sleep(3)
            os.execv(sys.executable, [sys.executable, os.path.abspath(__file__)] + sys.argv[1:])
        time.sleep(0.5)


found = call("search_entities", {"namePattern": "procedural-table"})
found = found.get("result", "") if isinstance(found, dict) else found
ids = {n: u for n, u in re.findall(r"(procedural-table(?:-collider)?) \(UUID: (\d+)\)", found)}
table_id, coll_id = ids.get("procedural-table"), ids.get("procedural-table-collider")
if not table_id:
    sys.exit("No entity called 'procedural-table' in the open scene.")

insp = scene({"schema_version": 1, "action": "inspect", "entities": [x for x in (table_id, coll_id) if x]})
ent = {e["entity_id"]: e for e in insp["entities"]}
tab = ent[table_id]
ov = {o["node_id"]: o["value"]["value"] for o in tab["pod"]["parameter_overrides"]}
vals = {"seed": ov.get("355", 7), "table_type": ov.get("356", 0), "shuffle_type": ov.get("357", 0),
        "shuffle_dimensions": ov.get("358", 0)}
pos = tab["local_transform"]["translation"]["value"]
if any(abs(v) > 1e-6 for v in tab["local_transform"]["rotation_euler"]["value"]):
    sys.exit("The table is rotated; this script only handles an unrotated table.")

# keep the collider's parameters equal to the table's
if coll_id:
    edits = [{"op": "set_parameter", "entity_id": coll_id, "name": k, "value": {"kind": "Int32", "value": int(v)}}
             for k, v in vals.items()]
    r = scene({"schema_version": 1, "action": "edit", "expected": insp["stamp"],
               "preconditions": [{"entity_id": coll_id, "fingerprint": ent[coll_id]["fingerprint"]}], "edits": edits})
    print("collider params:", r.get("code"), r.get("message", ""))
else:
    print("note: no procedural-table-collider entity found; props have nothing to rest on")

# evaluate the layout graph's derived params
gen_props.build()
env = {"min": min, "max": max, "abs": abs, "floor": math.floor, "ceil": math.ceil, "round": round,
       "sin": math.sin, "cos": math.cos, "tan": math.tan, "pow": pow, "sqrt": math.sqrt}
vals.update({"prop_seed": a.prop_seed, "prop_count": a.count, "edge_margin": a.margin,
             "drop_height": a.drop, "tumble": a.tumble})
for name, p in gt.PARAMS.items():
    if "expr" in p:
        env[name] = vals[name] = float(eval(p["expr"], {"__builtins__": {}}, {**env, **vals}))
    else:
        vals.setdefault(name, p["default"])
base = json.load(open(os.path.join(HERE, "props_base.json")))
prop_ids = list(base)
place = [dict(prop=int(vals[f"prop{i}"]), x=vals[f"x{i}"], z=vals[f"z{i}"], yaw=vals[f"yaw{i}"],
              tx=vals[f"tilt_x{i}"], tz=vals[f"tilt_z{i}"]) for i in range(a.count)]

def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)


def qaxis(ax, ang):
    s = math.sin(ang / 2)
    return (math.cos(ang / 2), ax[0] * s, ax[1] * s, ax[2] * s)


def q_from_euler(e):
    return qmul(qaxis((0, 0, 1), e[2]), qmul(qaxis((0, 1, 0), e[1]), qaxis((1, 0, 0), e[0])))


def euler_from_q(q):
    w, x, y, z = q
    return [math.atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z), math.asin(max(-1.0, min(1.0, -2*(x*z - w*y)))),
            math.atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z)]


bounds = json.load(open(os.path.join(HERE, "props_bounds.json")))


def rotate(q, v):
    w, x, y, z = q
    cx, cy, cz = y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]
    dx, dy, dz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (v[0] + 2 * (w * cx + dx), v[1] + 2 * (w * cy + dy), v[2] + 2 * (w * cz + dz))


def corners(pid):
    lo, hi = bounds[pid]["min"], bounds[pid]["max"]
    return [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]


def tilted(p):
    pid = prop_ids[p["prop"] % len(prop_ids)]
    q = qmul(qaxis((0, 0, 1), p["tz"]), qmul(qaxis((1, 0, 0), p["tx"]), q_from_euler(base[pid]["rot"])))
    return pid, [rotate(q, v) for v in corners(pid)]


def centre_and_radius(p):
    """Horizontal centre of the prop's box relative to its origin (some meshes have a far-off pivot) and a yaw-proof radius around it."""
    pid, pts = tilted(p)
    cx = (min(c[0] for c in pts) + max(c[0] for c in pts)) / 2
    cz = (min(c[2] for c in pts) + max(c[2] for c in pts)) / 2
    return (cx, cz), max(math.hypot(c[0] - cx, c[2] - cz) for c in pts) + 0.01


def radius(p):
    p["c"], r = centre_and_radius(p)
    return r


ROUND = vals["top_round"] > 0.5
lim_x, lim_z = vals["W"] / 2 - 0.02, vals["D"] / 2 - 0.02
lim_r = vals["top_r"] - 0.02
for p in place:
    p["r"] = radius(p)


def inside_clamp(p):
    if ROUND:
        d = math.hypot(p["x"], p["z"])
        room = max(0.0, lim_r - p["r"])
        if d > room:
            k = room / d if d else 0.0
            p["x"] *= k
            p["z"] *= k
    else:
        rx, rz = min(p["r"], lim_x), min(p["r"], lim_z)
        p["x"] = max(-lim_x + rx, min(lim_x - rx, p["x"]))
        p["z"] = max(-lim_z + rz, min(lim_z - rz, p["z"]))


def relax(items):
    for _ in range(800):
        moved = 0.0
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                A, B = items[i], items[j]
                dx, dz = B["x"] - A["x"], B["z"] - A["z"]
                d = math.hypot(dx, dz) or 1e-6
                gap = A["r"] + B["r"] + 0.01 - d
                if gap > 0:
                    A["x"] -= dx / d * gap / 2
                    A["z"] -= dz / d * gap / 2
                    B["x"] += dx / d * gap / 2
                    B["z"] += dz / d * gap / 2
                    moved += gap
        for p in items:
            inside_clamp(p)
        if moved < 1e-5:
            return True
    return False


def overlap(items):
    worst, who = 0.0, None
    for i in range(len(items)):
        tot = 0.0
        for j in range(len(items)):
            if i != j:
                d = math.hypot(items[i]["x"] - items[j]["x"], items[i]["z"] - items[j]["z"])
                tot += max(0.0, items[i]["r"] + items[j]["r"] - d)
        if tot > worst + 1e-9:
            worst, who = tot, i
    return worst, who


# squeeze them in; if they cannot all fit without overlapping, send the biggest offender back home instead
excluded = [p for p in place if base[prop_ids[p["prop"] % len(prop_ids)]]["tag"] in cfg.get("exclude_tags", [])]
placed = [p for p in place if p not in excluded]
parked = list(excluded)
while True:
    relax(placed)
    worst, who = overlap(placed)
    if worst < 0.003 or not placed:
        break
    parked.append(placed.pop(who))
place = placed
if parked:
    print("left at home (excluded or did not fit):", ", ".join(base[prop_ids[p["prop"] % len(prop_ids)]]["tag"] for p in parked))


ops = []
for p in place:
    pid = prop_ids[p["prop"] % len(prop_ids)]
    q = qmul(qaxis((0, 1, 0), p["yaw"]),
             qmul(qmul(qaxis((0, 0, 1), p["tz"]), qaxis((1, 0, 0), p["tx"])), q_from_euler(base[pid]["rot"])))
    e = euler_from_q(q)
    lowest = min(rotate(q, v)[1] for v in corners(pid))
    ox, _, oz = rotate(qaxis((0, 1, 0), p["yaw"]), (p["c"][0], 0.0, p["c"][1]))
    wp = [pos[0] + p["x"] - ox, pos[1] + vals["H"] + a.drop - lowest, pos[2] + p["z"] - oz]
    ops.append({"entityID": pid, "componentName": "TransformComponent", "propertyName": "Translation",
                "value": " ".join(f"{v:.6g}" for v in wp)})
    ops.append({"entityID": pid, "componentName": "TransformComponent", "propertyName": "Rotation",
                "value": " ".join(f"{v:.6g}" for v in e)})
for p in parked:
    pid = prop_ids[p["prop"] % len(prop_ids)]
    ops.append({"entityID": pid, "componentName": "TransformComponent", "propertyName": "Translation",
                "value": " ".join(f"{v:.6g}" for v in base[pid]["home"])})
    ops.append({"entityID": pid, "componentName": "TransformComponent", "propertyName": "Rotation",
                "value": " ".join(f"{v:.6g}" for v in base[pid]["rot"])})
res = call("batch_set_entity_property", {"operations": ops})
print(f"table seed {int(vals['seed'])} top {vals['W']:.2f} x {vals['D']:.2f} round={int(vals['top_round'])}; "
      f"{len(place)} on the table, {len(parked)} left at home:", res.get("result", res) if isinstance(res, dict) else res)
print("Edits are live and unsaved - press Ctrl+S in the editor to keep them.")
