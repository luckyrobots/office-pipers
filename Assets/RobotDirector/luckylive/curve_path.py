# LuckyLive binding for CurvePath (see LuckyLive's README). LuckyLive finds it in the open
# project's Assets by itself. In Edit mode it writes the same graph parameters as
# CurvePath.UpdateCurve does in Play, so each curve follows its child waypoints.
from luckylive import binding, helpers

MAX_WAYPOINTS = 8  # CurveFromAnchors takes 2-8 anchors
ARROW_AXIS = [0.0, 1.0, 0.0]  # SM_CurveWaypoint's tip points along local +Y


@binding("CanTake.CurvePath", fields=("FollowArrows", "Tension", "Closed", "Radius"), status_field="SyncStatus")
def curve_path(t):
    if len(t.children) < 2:
        return t.status("add at least 2 child waypoints")
    if not t.has_pod:
        return t.status("needs a Procedural Mesh using CurvePath.hpodgraph")

    waypoints = t.children[:MAX_WAYPOINTS]
    follow = helpers.as_bool(t.fields.get("FollowArrows"))
    t.set_param("anchor_count", "Int32", len(waypoints))
    t.set_param("tension", "Float", helpers.as_float(t.fields.get("Tension"), 1.0))
    t.set_param("closed", "Bool", helpers.as_bool(t.fields.get("Closed")))
    if "Radius" in t.fields:
        t.set_param("radius", "Float", helpers.as_float(t.fields["Radius"], 0.002))
    for i, w in enumerate(waypoints):
        # Children's translations are local to the curve entity, which is the graph's space
        t.set_param(f"anchor_{i}", "Float3", helpers.translation(w))
        # A zero axis tells the graph to use its automatic tangent
        t.set_param(f"axis_{i}", "Float3", helpers.rotate_euler(ARROW_AXIS, helpers.rotation(w)) if follow else [0.0, 0.0, 0.0])

    extra = len(t.children) - MAX_WAYPOINTS
    t.status(f"following {len(waypoints)} waypoints" + (f" ({extra} more ignored: the graph takes at most {MAX_WAYPOINTS})" if extra > 0 else ""))
