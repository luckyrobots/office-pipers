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

ap = argparse.ArgumentParser()
ap.add_argument("--count", type=int, default=10)
ap.add_argument("--prop-seed", type=int, default=1)
ap.add_argument("--tumble", type=float, default=0.0)
ap.add_argument("--drop", type=float, default=0.15)
ap.add_argument("--margin", type=float, default=0.07)
ap.add_argument("--watch", action="store_true", help="keep running and re-lay the props whenever the table seed changes")
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


ctx = call("authoring_state", {})["native"]["context_id"]
if not ctx:
    if a.watch:
        import time
        time.sleep(3)
        os.execv(sys.executable, [sys.executable, os.path.abspath(__file__)] + sys.argv[1:])
    sys.exit("The editor has no editable scene open yet.")


def scene(req):
    return call("scene_authoring", {"context_id": ctx, "request": req})["native"]


if a.watch:
    # Keep running: whenever the table's seed / shuffles / type change (edit mode only), re-run this script once.
    import subprocess, time
    child = [sys.executable, os.path.abspath(__file__)] + [x for x in sys.argv[1:] if x != "--watch"]
    last = None
    print("Watching the procedural-table seed. Leave this window open; Ctrl+C to stop.")
    while True:
        try:
            if call("harness/get_scene_state_mode", {}).get("name") == "Edit":
                txt = call("search_entities", {"namePattern": "procedural-table"})
                txt = txt.get("result", "") if isinstance(txt, dict) else txt
                tid = dict((n, u) for n, u in re.findall(r"(procedural-table(?:-collider)?) \(UUID: (\d+)\)", txt)).get("procedural-table")
                if tid:
                    snap = scene({"schema_version": 1, "action": "inspect", "entities": [tid]})
                    sig = json.dumps(sorted((o["node_id"], o["value"]["value"]) for o in snap["entities"][0]["pod"]["parameter_overrides"]))
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

RADII = {"baseball": 0.05, "elephant": 0.065, "lubricant": 0.085, "cat_statue": 0.15, "apple": 0.055,
         "gnome": 0.13, "waterbottle": 0.09, "bucket": 0.13, "tongue": 0.13, "pliers": 0.1}


def radius(p):
    tg = base[prop_ids[p["prop"] % len(prop_ids)]]["tag"]
    return next((r for k, r in RADII.items() if k in tg), 0.1)


lim_x, lim_z = vals["W"] / 2 - 0.02, vals["D"] / 2 - 0.02
if vals["top_round"] > 0.5:
    lim_x = lim_z = vals["top_r"] * 0.9 - 0.02
for p in place:
    p["r"] = radius(p)
for it in range(600):
    moved = 0.0
    for i in range(len(place)):
        for j in range(i + 1, len(place)):
            A, B = place[i], place[j]
            dx, dz = B["x"] - A["x"], B["z"] - A["z"]
            d = math.hypot(dx, dz) or 1e-6
            gap = A["r"] + B["r"] + 0.01 - d
            if gap > 0:
                A["x"] -= dx / d * gap / 2
                A["z"] -= dz / d * gap / 2
                B["x"] += dx / d * gap / 2
                B["z"] += dz / d * gap / 2
                moved += gap
    for p in place:
        r = min(p["r"], 0.9 * min(lim_x, lim_z))
        p["x"] = max(-lim_x + r, min(lim_x - r, p["x"]))
        p["z"] = max(-lim_z + r, min(lim_z - r, p["z"]))
    if moved < 1e-5:
        break


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


ops = []
for p in place:
    pid = prop_ids[p["prop"] % len(prop_ids)]
    q = qmul(qaxis((0, 1, 0), p["yaw"]),
             qmul(qmul(qaxis((0, 0, 1), p["tz"]), qaxis((1, 0, 0), p["tx"])), q_from_euler(base[pid]["rot"])))
    e = euler_from_q(q)
    wp = [pos[0] + p["x"], pos[1] + vals["surf_y"], pos[2] + p["z"]]
    ops.append({"entityID": pid, "componentName": "TransformComponent", "propertyName": "Translation",
                "value": " ".join(f"{v:.6g}" for v in wp)})
    ops.append({"entityID": pid, "componentName": "TransformComponent", "propertyName": "Rotation",
                "value": " ".join(f"{v:.6g}" for v in e)})
res = call("batch_set_entity_property", {"operations": ops})
print(f"table seed {int(vals['seed'])} top {vals['W']:.2f} x {vals['D']:.2f} round={int(vals['top_round'])}; "
      f"{len(place)} props:", res.get("result", res) if isinstance(res, dict) else res)
print("Edits are live and unsaved - press Ctrl+S in the editor to keep them.")
