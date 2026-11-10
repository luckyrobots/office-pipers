#!/usr/bin/env python3
"""Automated takes: for each table seed, re-lay the props, wait for the cooks, press Play, let the physics settle,
check that every prop is resting ON the table, stop, and write a report.

    py run_takes.py --seeds 1-20 [--settle 6] [--shuffle-type 0] [--shuffle-dimensions 0]
    py run_takes.py --seeds 135,171,320 --report takes_report.json

Stop the watcher first (watch_table_props.bat): both edit the scene. The table's original seed is restored at the end.
The scene is left with unsaved edits; close without saving, or restore with undo, if you do not want the last take kept.
"""
import argparse, json, math, os, re, subprocess, sys, time, urllib.request
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_props, gen_table as gt

ap = argparse.ArgumentParser()
ap.add_argument("--seeds", default="1-10", help="e.g. 1-20 or 3,7,135")
ap.add_argument("--settle", type=float, default=4.0, help="minimum seconds of Play per take")
ap.add_argument("--max-settle", type=float, default=40.0, help="give up waiting for the props to come to rest after this long")
ap.add_argument("--shuffle-type", type=int, default=None)
ap.add_argument("--shuffle-dimensions", type=int, default=None)
ap.add_argument("--table-type", type=int, default=None)
ap.add_argument("--report", default=os.path.join(HERE, "takes_report.json"))
a = ap.parse_args()

seeds = []
for part in a.seeds.split(","):
    if "-" in part:
        lo, hi = part.split("-")
        seeds += list(range(int(lo), int(hi) + 1))
    else:
        seeds.append(int(part))

URL = "http://127.0.0.1:19331/mcp"
HDR = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}


def post(body, sid=None):
    h = dict(HDR)
    if sid:
        h["Mcp-Session-Id"] = sid
    with urllib.request.urlopen(urllib.request.Request(URL, data=json.dumps(body).encode(), headers=h), timeout=120) as r:
        return r.headers.get("Mcp-Session-Id"), r.read().decode()


sid, _ = post({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
    "protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "takes", "version": "1"}}})
post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)


def call(name, args):
    _, txt = post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": name, "arguments": args}}, sid)
    c = json.loads(txt)["result"]["content"][0]["text"]
    try:
        return json.loads(c)
    except Exception:
        return c


def fresh_ctx(timeout=30):
    """The scene's authoring context changes after every Play/Stop, so ask again each time."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            c = call("authoring_state", {})["native"].get("context_id")
            if c:
                return c
        except Exception:
            pass
        time.sleep(0.5)
    sys.exit("The editor has no editable scene open.")


def scene(req):
    r = call("scene_authoring", {"context_id": fresh_ctx(), "request": req})
    return r["native"]


def mode():
    return call("harness/get_scene_state_mode", {}).get("name")


def wait_mode(want, timeout=40):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if mode() == want:
            return True
        time.sleep(0.4)
    return False


def entity_ids():
    txt = call("search_entities", {"namePattern": "procedural-table"})
    txt = txt.get("result", "") if isinstance(txt, dict) else txt
    return {n: u for n, u in re.findall(r"(procedural-table(?:-collider)?) \(UUID: (\d+)\)", txt)}


ids = entity_ids()
TABLE, COLL = ids.get("procedural-table"), ids.get("procedural-table-collider")
if not TABLE or not COLL:
    sys.exit(f"Need both 'procedural-table' and 'procedural-table-collider' in the scene (found {list(ids)}).")
base = json.load(open(os.path.join(HERE, "props_base.json")))
prop_ids = list(base)


def inspect(*eids):
    r = scene({"schema_version": 1, "action": "inspect", "entities": list(eids)})
    return r, {e["entity_id"]: e for e in r["entities"]}


def table_state():
    _, e = inspect(TABLE)
    t = e[TABLE]
    ov = {o["node_id"]: o["value"]["value"] for o in t["pod"]["parameter_overrides"]}
    return ov, t["local_transform"]["translation"]["value"]


def set_table(values):
    r, e = inspect(TABLE)
    edits = [{"op": "set_parameter", "entity_id": TABLE, "name": k, "value": {"kind": "Int32", "value": int(v)}}
             for k, v in values.items()]
    out = scene({"schema_version": 1, "action": "edit", "expected": r["stamp"],
                 "preconditions": [{"entity_id": TABLE, "fingerprint": e[TABLE]["fingerprint"]}], "edits": edits})
    if out.get("code") not in ("applied", "unchanged"):
        raise RuntimeError(f"set table params failed: {out.get('code')} {out.get('message')}")


def collider_handle():
    s = call("get_entity_properties", {"entityID": COLL})["result"]
    m = re.search(r"StaticMesh Handle: (\d+)", s)
    return m.group(1) if m else None


def wait_cooks(old_handle, timeout=60):
    """Table cook via authoring_cooks. The collider has no render faces, so authoring_cooks always reports it as
    failed; instead wait for its collision mesh handle to change (it is rebuilt when its parameters change)."""
    t0 = time.time()
    ok = False
    while time.time() - t0 < timeout:
        r, e = inspect(TABLE)
        out = call("authoring_cooks", {"context_id": fresh_ctx(), "expected": r["stamp"],
                                       "preconditions": [{"entity_id": TABLE, "fingerprint": e[TABLE]["fingerprint"]}]})
        ready = out.get("ready") if isinstance(out, dict) else None
        if ready is None and isinstance(out, dict):
            ready = out.get("native", {}).get("ready")
        if ready is True or (isinstance(out, dict) and out.get("outcome") in ("completed", "ready", "succeeded")):
            ok = True
            break
        time.sleep(0.7)
    t1 = time.time()
    while time.time() - t1 < 20 and collider_handle() == old_handle:
        time.sleep(0.5)
    time.sleep(1.0)
    return ok, collider_handle() != old_handle


def table_geometry(vals):
    if not gt.PARAMS:
        gen_props.build()
    env = {"min": min, "max": max, "abs": abs, "floor": math.floor, "ceil": math.ceil, "round": round,
           "sin": math.sin, "cos": math.cos, "tan": math.tan, "pow": pow, "sqrt": math.sqrt}
    v = dict(vals)
    v.update({"prop_seed": 1, "prop_count": 10, "edge_margin": 0.07, "drop_height": 0.04, "tumble": 0.0})
    for name, p in gt.PARAMS.items():
        if "expr" in p:
            env[name] = v[name] = float(eval(p["expr"], {"__builtins__": {}}, {**env, **v}))
        else:
            v.setdefault(name, p["default"])
    return v


def prop_positions():
    out = {}
    for pid in prop_ids:
        s = call("get_entity_properties", {"entityID": pid})["result"]
        out[pid] = [float(x) for x in re.search(r"Translation: \(([^)]*)\)", s).group(1).split(",")]
    return out


if mode() != "Edit":
    call("harness/exit_play_mode", {})
    wait_mode("Edit")

orig_ov, tpos = table_state()
orig = {"seed": orig_ov.get("355", 7), "shuffle_type": orig_ov.get("357", 0), "shuffle_dimensions": orig_ov.get("358", 0)}
report = []
print(f"{len(seeds)} takes, settle {a.settle}s each. Original table seed {orig['seed']} will be restored.\n")
try:
    for n, seed in enumerate(seeds, 1):
        take = {"take": n, "seed": seed, "problems": []}
        t0 = time.time()
        try:
            want = {"seed": seed}
            if a.shuffle_type is not None: want["shuffle_type"] = a.shuffle_type
            if a.shuffle_dimensions is not None: want["shuffle_dimensions"] = a.shuffle_dimensions
            if a.table_type is not None: want["table_type"] = a.table_type
            old_handle = collider_handle()
            set_table(want)
            rl = subprocess.run([sys.executable, os.path.join(HERE, "relayout_props.py")], capture_output=True, text=True, timeout=120)
            if rl.returncode != 0 or "0 failed" not in rl.stdout:
                take["problems"].append("relayout: " + (rl.stdout + rl.stderr).strip()[-200:])
            m = re.search(r"(?m)^left at home \(excluded or did not fit\): (.*)", rl.stdout)
            parked = [t.strip() for t in m.group(1).split(",")] if m else []
            take["parked"] = parked
            ok_cook, collider_rebuilt = wait_cooks(old_handle)
            if not ok_cook:
                take["problems"].append("table cook not confirmed after 60 s")
            take["collider_rebuilt"] = collider_rebuilt
            ov, tpos = table_state()
            vals = table_geometry({"seed": ov.get("355", seed), "table_type": ov.get("356", 0),
                                   "shuffle_type": ov.get("357", 0), "shuffle_dimensions": ov.get("358", 0)})
            hx = (vals["top_r"] if vals["top_round"] > 0.5 else vals["W"] / 2)
            hz = (vals["top_r"] if vals["top_round"] > 0.5 else vals["D"] / 2)
            top_y = tpos[1] + vals["H"]
            take["table"] = {"W": round(vals["W"], 3), "D": round(vals["D"], 3), "round": int(vals["top_round"]), "top_y": round(top_y, 3)}
            start = prop_positions()
            call("harness/enter_play_mode", {})
            if not wait_mode("Play", 60):
                raise RuntimeError("never reached Play")
            # simulation can run slower than real time, so poll until everything is still instead of sleeping a fixed time
            t_play = time.time()
            prev = None
            still = 0
            end = start
            while time.time() - t_play < a.max_settle:
                time.sleep(1.0)
                if mode() != "Play":
                    take["problems"].append("Play ended early (script or crash?)")
                    break
                end = prop_positions()
                if prev is not None and all(max(abs(end[p][i] - prev[p][i]) for i in range(3)) < 0.004 for p in prop_ids):
                    still += 1
                    if still >= 2 and time.time() - t_play >= a.settle:
                        break
                else:
                    still = 0
                prev = end
            else:
                take["problems"].append(f"not settled after {a.max_settle:.0f}s")
            take["settle_seconds"] = round(time.time() - t_play, 1)
            end2 = end
            moved = 0
            for pid in prop_ids:
                tag = base[pid]["tag"]
                if tag in parked:
                    continue
                x, y, z = end[pid]
                if not all(math.isfinite(c) for c in (x, y, z)):
                    take["problems"].append(f"{tag}: non-finite position")
                    continue
                dx, dz = x - tpos[0], z - tpos[2]
                inside = (abs(dx) <= hx + 0.03 and abs(dz) <= hz + 0.03) if vals["top_round"] < 0.5 else (math.hypot(dx, dz) <= hx + 0.03)
                if not inside:
                    take["problems"].append(f"{tag}: off the table ({dx:+.2f}, {dz:+.2f} from centre)")
                if y < top_y - 0.03:
                    take["problems"].append(f"{tag}: below the top (y {y:.2f} vs top {top_y:.2f})")
                elif y > top_y + 0.45:
                    take["problems"].append(f"{tag}: stacked/too high (y {y:.2f})")
                if max(abs(end2[pid][i] - end[pid][i]) for i in range(3)) > 0.01:
                    take["problems"].append(f"{tag}: still moving")
                if abs(y - start[pid][1]) > 1e-4:
                    moved += 1
            if moved == 0:
                take["problems"].append("no prop moved: physics may not have run")
            take["props_moved"] = moved
        except Exception as ex:
            import traceback
            take["problems"].append(f"exception: {ex!r} at line {traceback.extract_tb(sys.exc_info()[2])[-1].lineno}")
        finally:
            try:
                if mode() != "Edit":
                    call("harness/exit_play_mode", {})
                    wait_mode("Edit")
            except Exception as ex:
                take["problems"].append(f"could not return to Edit: {ex}")
        take["ok"] = not take["problems"]
        take["seconds"] = round(time.time() - t0, 1)
        report.append(take)
        flag = "PASS" if take["ok"] else "FAIL"
        print(f"take {n:2d} seed {seed:4d} {flag} {take.get('table', '')} {take['seconds']}s" + (f" (parked: {', '.join(take['parked'])})" if take.get("parked") else ""))
        for pr in take["problems"]:
            print("      -", pr)
        json.dump(report, open(a.report, "w", encoding="utf-8"), indent=1)
finally:
    try:
        if mode() != "Edit":
            call("harness/exit_play_mode", {})
            wait_mode("Edit")
        set_table(orig)
        subprocess.run([sys.executable, os.path.join(HERE, "relayout_props.py")], capture_output=True, text=True, timeout=120)
        print(f"\nRestored table seed {orig['seed']} and re-laid the props.")
    except Exception as ex:
        print("could not restore the table seed:", ex)

passed = sum(1 for t in report if t["ok"])
print(f"\n{passed}/{len(report)} takes passed. Report: {a.report}")
