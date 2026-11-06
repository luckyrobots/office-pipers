# LuckyLive binding for LD_Event (see LuckyLive's README). In Edit mode it shows the child icon for
# the event's Action and hides the others: LD_Rotate for Rotate, LD_Open for Open Grabber, LD_Close
# for Close and Set Grabber. The editor can't hide a mesh from outside Play, so a hidden icon gets
# SM_LD_Hidden (a speck) and a shown one gets its own mesh back. In Play, RobotDirector sets the
# icons' Visible instead (LD_Event.ShowIcons).
from luckylive import binding, helpers

HIDDEN = "5600224601578857925"  # assets/events/SM_LD_Hidden.hsmesh
ICONS = {  # child tag: its own StaticMesh handle
    "LD_Rotate": "2347882247835264959",  # SM_LD_Rotate
    "LD_Open": "7006161624537359908",  # SM_LD_Open_1
    "LD_Close": "1426231606741906005",  # SM_LD_Close_1
}
SHOWN = {0: "LD_Rotate", 1: "LD_Open", 2: "LD_Close", 3: "LD_Close"}  # LD_Event.Action: icon


@binding("LuckyExporter.LD_Event", fields=("Action",))
def ld_event(t):
    shown = SHOWN.get(helpers.as_int(t.fields.get("Action")), "LD_Rotate")
    for child in t.children:
        if child["name"] not in ICONS:
            continue
        want = ICONS[child["name"]] if child["name"] == shown else HIDDEN
        have = helpers.mesh_handle(t.properties(child))
        if have is not None and have != want:
            t.set_property(child, "StaticMesh", "MeshHandle", want)
