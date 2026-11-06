# RobotDirector

A drop-in package for Lucky Engine that scripts the OpenArm pedestal robot and draws editable
curves in the scene. It contains:

| Tool | What it does |
|---|---|
| **CurvePath** | A smooth tube through the child entities of the CurvePath entity, built by a procedural graph. Move the children and the curve follows. |
| **CurveFollower** | Moves one robot hand along a CurvePath at a constant speed. |
| **RobotDirector** + **RobotAction** | Plays a list of steps one after another: move a hand to an entity, set a graph input, or wait. |
| **RobotInput** enum | Every input name of the OpenArm motion graph, so code doesn't use raw strings. |

## Contents

```
RobotDirector/
├── package.manifest            Every asset file with its fixed ID and type (LuckyLive registers them)
├── Check-Package.ps1           Optional, for changing the package: checks it's self-contained
├── prefabs/
│   ├── LD_Path.hprefab         A curve with 3 waypoints: drag this into a scene
│   ├── LD_Target.hprefab       A director waypoint: one entity, the arrow mesh with LD_Waypoint; its pivot (the arrow's tip) is where the fingertips go, along the arrow (+Y)
│   ├── LD_Event.hprefab        A director event ("!" with Rotate, Close and Open icons, and LD_Event): turns the hand or works the gripper in place
│   ├── LD_Goal.hprefab         A claw marker (one entity, the ring mesh with LD_Waypoint; its centre is where the fingertips go, pointing +Y): a director waypoint or RobotAction Move target
│   └── LuckyDirector.hprefab   A RobotDirector entity with a logo and label
├── assets/
│   ├── curvePath/
│   │   ├── CurvePath.hpodgraph     Procedural graph: anchors → Bézier curve → tube
│   │   ├── CurvePath.cs            Script that feeds the child positions to the graph, with the Path Task settings for RobotDirector
│   │   └── CurvePathEditorSync.cs  Starts LuckyLive on Play if it isn't running, or warns if it isn't installed
│   ├── waypoint/
│   │   ├── SM_LD_Waypoint.glb / .hsmesh     Waypoint arrow mesh, pointing +Y with its pivot at the tip
│   │   └── M_CurveWaypoint.hmaterial        Material shared by all the package's meshes
│   ├── target/SM_LD_Target.glb / .hsmesh    Ring mesh for LD_Goal, facing +Y (the fingers come through it), pivot at its centre (the target)
│   ├── events/                              Meshes for LD_Event: SM_LD_Event ("!", 9 cm), SM_LD_Rotate, SM_LD_Close_1, SM_LD_Open_1, SM_LD_Hidden (empty, for hidden icons)
│   └── logo/SM_Logo.glb / .hsmesh           Logo mesh for LuckyDirector
├── luckylive/
│   ├── curve_path.py           LuckyLive binding: LuckyLive finds it here and updates every CurvePath in Edit mode
│   └── ld_event.py             LuckyLive binding: shows each LD_Event's icon for its Action in Edit mode
└── tools/
    ├── LuckyDirector.cs        RobotDirector, the RobotInput enum and its helpers
    ├── LD_RobotAction.cs       RobotAction (one step)
    ├── LD_CurveFollower.cs     CurveFollower, and CurveMotion (the follow motion, shared with RobotDirector)
    ├── components/
    │   ├── LD_Waypoint.cs      LD_Waypoint: a point a RobotDirector moves the hand to
    │   └── LD_Event.cs         LD_Event: turn the hand or open, close or set the gripper in place, waiting or Async
    └── RobotController-API.md  Reference for every controller input
```

All scripts are in the `CanTake` namespace.

## Using it in a project

Nothing to install. With [LuckyLive](../LuckyLive/README.md) running:

1. Copy the `RobotDirector` folder anywhere under the project's `Assets` folder. The editor can
   stay open.
2. Within a few seconds LuckyLive registers the package's assets (its log says
   `registered 12 asset(s) of RobotDirector`). The editor compiles the scripts on its own.
3. Drag `prefabs/LD_Path.hprefab` into the scene. Its curve follows the waypoints in Edit
   mode while LuckyLive runs.

The editor needs its MCP server on, for example `LuckyEngine.exe --mcp=19330`. LuckyLive finds it
on its own.

**Why the assets need registering.** Assets refer to each other by numeric ID, and each project
keeps its own list of IDs in `Assets/AssetRegistry.hzr`. A copied file normally gets a new random
ID, so the prefab would lose its graph, mesh and material. LuckyLive gives every file listed in
`package.manifest` its fixed ID: it adds the entries to `AssetRegistry.hzr` (keeping the previous
file as `AssetRegistry.hzr.luckylive-backup`) and has the open editor reread it. It only touches
this package's entries, and leaves the list alone if the package is in the project twice, saying
so in its log. Once registered, the project keeps the IDs, so LuckyLive isn't needed for that
again.

Have LuckyLive running before you copy the folder in, or at least before you use the prefabs. If
the editor gives the files random IDs first, LuckyLive still adds the fixed ones, which are what
the prefabs use. The random entries stay in the list as second IDs for the same files.

The `.cs` scripts need no registration, because the engine compiles every `.cs` file under
`Assets` and finds scripts by class name.

**After moving files inside the folder:** if you move or rename any asset file, update its `path`
in `package.manifest`. LuckyLive re-registers it at its new path. If the project was open in the
editor at the time, close and reopen it: the open editor can't move an ID to a new path, and
LuckyLive's log says so.

**Updating a project that has an older copy:** replace the files, then close and reopen the
project if any file was renamed (for example `LuckyyDirector.hprefab`, now `LuckyDirector.hprefab`).

**Adding a file to the package:** a copied prefab or mesh still refers to the originals' IDs. Add it
to `package.manifest` with a new ID, point its references at package assets, and run
`Check-Package.ps1` (right-click → **Run with PowerShell**). It changes nothing, and lists any
reference to a file outside the package, any prefab that claims another prefab's ID, duplicated
nested prefab entities, and any asset file missing from the manifest.

`package.manifest` is JSON, but don't rename it to `.json`, because the editor would import it as
a Motion asset.

## CurvePath

The waypoints are the **child entities** of the CurvePath entity, in name order (`Waypoint_2`
comes before `Waypoint_10`). To add a point, duplicate a waypoint and rename it. The graph takes
at most 8; any beyond that are ignored, with a warning in the log.

| Field | Meaning |
|---|---|
| Follow Arrows / Auto Tangents | Pass through each waypoint in the direction its arrow points, or ignore rotation and smooth between neighbours |
| Tension | Handle length at each waypoint. Higher holds each direction for longer |
| Closed Loop / Open | Join the last waypoint back to the first |
| Radius | Tube radius, in metres |
| Visible During Sim | Show the curve and its waypoints in Play; the waypoints always match the curve. On by default; off hides both. Can be switched while playing, and a RobotDirector's Task Visibility can override it |
| LuckyLive: Auto-Start, Sync Status | See below |

In Play the curve always follows its waypoints, since the script rebuilds it whenever one moves.

Other scripts can call `SampleLocal()` to get the curve points in the CurvePath's local space, and
`SampleRotationsLocal()` for the waypoints' rotations blended along it, point for point.
CurveFollower uses both.

### Updating the curve in Edit mode

Scripts only run in Play, so in Edit mode [LuckyLive](../LuckyLive/README.md) keeps the curve up
to date. LuckyLive is a small app shared by every package that needs Edit-mode updates.

- **What it needs:** `LuckyLive.exe` running, and the editor started with its MCP server on, for
  example `LuckyEngine.exe --mcp=19330`, which listens on port 19331. Nothing to install: LuckyLive
  finds this package's `luckylive/curve_path.py` in the open project on its own.
- **How it starts:** run `LuckyLive.exe`, and tick **Start with Windows** in its window to have it
  there at every logon. With **Auto-Start LuckyLive** on (the default), pressing Play also starts it
  (in the tray) if it isn't already running. If it has never been run on this machine, or has been
  moved, Play logs a `[CurvePath]` warning saying so instead.
- **What it does:** about four times a second, it reads every CurvePath, its children and its
  fields, and writes any graph parameters that changed. Each update is one Undo step.
- **Feedback:** the read-only **Sync Status** field on each CurvePath shows what LuckyLive last
  reported, such as `Running (LuckyLive pid 4364, since 15:37): following 3 waypoints`, or what's
  missing. LuckyLive's window shows the editors with their ports, whether each is connected,
  how many curves it follows, and the log. **Disconnect** there pauses the updates for one editor,
  and Sync Status then says so.
- **Changing the binding:** edit `luckylive/curve_path.py` in the project. LuckyLive reloads it
  within a couple of seconds.

Without LuckyLive or MCP, the curve still updates in Play, and stays as it was last left in Edit
mode.

To stop LuckyLive, quit it from its window or tray icon. To pause it for one editor only, use
**Disconnect** in its window.

## CurveFollower

Put it on any entity and set **Curve** to a CurvePath and **Robot** to the robot root (the entity
with the RobotController and MujocoScene components).

| Field | Meaning |
|---|---|
| Enabled | Off: the follower does nothing and the hand stays put. On again while playing: the hand eases back onto the curve |
| Curve, Robot | As above |
| Arm | The hand that follows the curve. The other holds its starting position |
| Speed | Metres per second along the curve |
| Ping-Pong / Loop | Run back and forth, or jump back to the start (wrap, on a closed curve) |
| Reverse / Forward | Start from the last waypoint and travel toward the first. Switching while playing turns the hand around |
| Approach Time | Seconds to ease the hand onto the start of the curve |
| Solve Orientation / Position Only | Also control the hand's orientation, or leave the wrist free |
| Orientation Source | **From Waypoints** (default): the hand matches each waypoint's arrow as it passes, blending in between, the same way as an LD_Waypoint: the fingers point the way the arrow points (+Y) and the jaws open along its X. A fresh LD_Path's arrows point down (fingers down). **Fixed**: hold Hand Orientation |
| Hand Orientation | Fixed: world XYZ degrees. From Waypoints: an extra rotation in each waypoint's frame, on top of its arrow |
| High Accuracy / Normal | More solver iterations, so the hand lags less |

On Play it eases the hand to the start of the curve, then moves the target along it every frame.
Everything can be changed while playing:

- **Editing the curve:** the hand continues from the nearest point on the new curve, moving there
  at no more than 0.25 m/s rather than snapping.
- **Changing Arm:** holds the old hand where it is and eases the new one onto the curve.

## RobotDirector and RobotAction

1. Make an empty entity, for example `Tasks`, and parent the steps under it. Name them
   `1_Goal`, `2_Path`, `3_Target` and so on: they play in name order (`2` comes before `10`), and
   two steps with the same name are warned about, because their order is then arbitrary. Each
   step can be any of these; the script can be on the step or one of its children, so a prefab
   instance can go in directly:
   - **A waypoint:** an `LD_Goal` or `LD_Target` prefab, or any entity with an **LD_Waypoint**
     script (both prefabs carry one, so each instance can be adjusted). A marker without the script
     moves with LD_Waypoint's defaults (2 s, From Waypoint, no waits, no grab). LuckyEngine's prefab
     editor can't load project scripts and may crash when you open or click a prefab that has one:
     adjust the instances in the scene instead. The director waits its **Wait Start**, eases the hand to its world position over its
     **Duration**, then waits its **Wait End**. Its Arm, Solve Orientation and Orientation fields
     work like a RobotAction Move; **From Waypoint** turns the hand to match the prefab's rotation: the fingers point along its +Y (the way the arrow points) and the jaws open along its X. Both prefabs start turned 180° about X, so a fresh one asks for the Panda's home pose (fingers down). **Grabber Action** (Open, Close, or Set to **Grabber Value**, from 0 closed to 1 fully open on any robot) moves that arm's gripper while the arm moves: it starts **Grabber Delay** seconds after the move starts and takes **Grabber Time** to open or close (Delay 0 and Time = Duration spread it across the whole move). It runs on its own timer and never holds up the tasks. **Immediate** carries the hand straight on from the previous waypoint without stopping there: it travels in a straight line and keeps its speed through the previous waypoint, still arriving on time. That needs no Wait End on the previous waypoint, no Wait Start on this one, the same Arm, and no other task between them; otherwise it sets off smoothly from rest, still in a straight line. Speed carries through a waypoint, but the direction changes there at once, so a sharp corner is only softened by how the arm follows.
   - **A random grab:** an `Ld_DuckGrabber` prefab, or any entity with an **LD_DuckGrabber** script.
     It's a waypoint with the same settings, except that when its turn comes (after **Wait Start**)
     it picks a random mesh inside **Box** that is easy to grab: on its side or upright (local +Y within
     **Pose Tolerance** of horizontal or straight up), with nothing else in the volume within
     **Clearance** of where the fingers land, and within **Max Reach** of the robot seen from above. The hand always comes from straight above: only its turn
     about the vertical follows the mesh, with the jaws closing around a duck's neck. **Offset Rotation**
     adds to that (Y turns about the fingers). **Grip Point** and **Upright Grip Point** are where the
     fingertips go for each pose, in the mesh's own unscaled coordinates: by default the rubber duck's
     neck, with **Side Jaw Angle** turning the jaws to fit a duck on its side. **Offset Position** adjusts that in metres, with +Y down along the fingers (so -Y
     hovers above) and X the way the jaws open. **Same Target As** points at an earlier LD_DuckGrabber
     and reuses its pick and hand rotation, so one task can hover, the next descend and close, and
     another lift. **Follow Target** re-aims the hand at the mesh every frame until the gripper closes,
     so it stays on the grip point if the mesh rolls or settles. **Fake Hold** locks the mesh to the hand
     from when the gripper starts closing until it starts opening, and on Panda stops the fingers where
     they touch it, so it can't slip or twist. It applies if the hand is within 4 cm of the grip point as
     the gripper closes; otherwise it warns and leaves the mesh to the fingers' friction. Grabber Action
     on any later task or an LD_Event works the hold too, until the mesh is released. **Box** is any entity with the default cube mesh, which can
     be rotated and hidden. A mesh counts as inside when its entity's origin is. The robot, the Task
     List, the director and Box are never candidates; **Tag Filter** narrows the rest by name, for
     example `rubber_duck`. With no candidates it warns and skips the move.
     **Target Marker** shows the result during Play: a label with the mesh's name and the position
     and rotation including the offsets, facing the scene camera. It stays until this task picks
     again, and **Marker Size** sets its size. **Marker Lines** adds axes at the point the hand is sent
     to, turned to its rotation, and lines from there to the mesh and the label. A director's **Task
     Visibility** of **Hide All But Labels** hides every task but keeps these labels.
   - **An event:** an `LD_Event` prefab, or any entity with an **LD_Event** script. It works the
     hand where it is, without moving the arm. Each event does one **Action**, starting after
     **Wait Start** and taking **Time** seconds:
     - **Rotate** turns the end of the arm about the fingers' axis by **Amount** degrees (negative
       the other way). Turns over 90° go in 90° steps, and the wrist's joint limits still apply
       (about ±166° on the Panda).
     - **Open Grabber** / **Close Grabber** open or close that arm's gripper fully (Time 0 snaps).
     - **Set Grabber** moves it to **Amount**, from 0 closed to 1 fully open on any robot.

     The event's child icons show its Action: **LD_Rotate** for Rotate, **LD_Open** for Open
     Grabber, **LD_Close** for Close and Set Grabber. LuckyLive keeps them in step in Edit mode
     (`luckylive/ld_event.py` hides the others by giving them the empty `SM_LD_Hidden` mesh), and
     the director sets them when Play starts.

     **Is Async** off (Wait): the next task waits for the action to finish, then Wait End. On
     (Async): the next task starts after Wait End while the action plays out. A gripper moves on
     its own timer. A turn still under way when the next waypoint (or marker) starts is carried by
     that move: the hand ends at the waypoint's orientation turned by Amount about the fingers'
     axis, turning as it travels over the move's Duration (the short way, so up to 180°). With
     Position Only, the turn is from the hand's orientation as the move starts. A curve or
     RobotAction next doesn't carry it, and takes the arm over.
   - **A curve:** an `LD_Path` prefab, or any CurvePath. The hand eases onto the start of the curve
     and follows it once to the end, using the same motion as CurveFollower. Its settings are in the
     **Path Task** group of the CurvePath script itself: Arm, Speed, Reverse / Forward, Approach
     Time, Solve Orientation, Orientation Source, Hand Orientation (as in CurveFollower), Wait
     Start and Wait End, and the same **Grabber Action / Value / Delay / Time** as a waypoint:
     Grabber Delay counts from the start of the task (the approach onto the curve), so Delay =
     Approach Time grabs as the hand sets off along it. CurveFollower ignores the Path Task group
     and uses its own settings.
   - **A RobotAction** (Move, Set Input or Wait), on an empty entity.

2. Add a **RobotDirector** to any entity and set **Robot** to the robot root.
3. Set **Task List** to the `Tasks` entity. (A single entity field: the editor doesn't keep
   entities picked inside an array.)

| RobotDirector field | Meaning |
|---|---|
| Robot | The robot root |
| Task Visibility | **Per Task** (default): each task as it's set. **Show All** / **Hide All**: every curve (its Visible During Sim) and waypoint marker in the Task List, in Play; Hide All also hides the director's logo. Can be changed while playing |
| Task List | The entity whose children are the steps, in name order |
| Loop / Once | Start again from the first step after the last |
| Tracking / Seeded | Tracking_Mode for both arms. Keep Tracking on unless an arm is stuck in a bad pose |
| High Accuracy / Normal | High_Accuracy for both arms |

On Play, the director holds both hands where they are and starts the first step.

Each RobotAction is one of three types. Every step has a **Hold**: seconds to wait after it before
the next one starts. Steps with a Hold of 0 run in the same frame as the next step.

- **Move:** eases **Arm** to the world position of **Target** over **Duration** seconds,
  optionally with a fixed **Orientation**. The director waits Duration + Hold.
- **Set Input:** writes one graph input picked from the dropdown, using the matching value field
  (Bool, Float or Vector Value). For a Vector3 input, **Vector Source** uses an entity's world
  position instead. The director waits Hold.
- **Wait:** waits Hold.

**Example: lower the left hand and lift it again**

1. Set Input, `LeftArm_Pole_Target`, Vector Source = `ElbowPole`, Hold 0
2. Set Input, `LeftArm_Pole_Weight`, Float Value 0.5, Hold 0
3. Move, Left Arm, Target = `Above`, Duration 2
4. Move, Left Arm, Target = `Down`, Duration 1, Hold 0.5
5. Move, Left Arm, Target = `Above`, Duration 1

Don't run CurveFollower and RobotDirector on the same robot at once, because they write the same
inputs.

## The motion graph in brief

The scripts drive `MG_OpenArmPedestal.hpodgraph` through the robot's `RobotControllerComponent`.
They work with any robot whose graph has these inputs (`<Arm>` is `LeftArm` or `RightArm`):

| Input | Type | Meaning |
|---|---|---|
| `Start` | Trigger | Record the current pose and start an eased move |
| `Duration` | Float | Length of that move, in seconds. 0 freezes the robot |
| `<Arm>_Target_Position` | Vector3 | Hand target, world space, metres |
| `<Arm>_Target_Orientation` | Vector3 | Hand orientation, world XYZ degrees |
| `<Arm>_Solve_Orientation` | Bool | Include orientation in the IK |
| `<Arm>_Pole_Target` | Vector3 | Point the elbow bends toward |
| `<Arm>_Pole_Weight` | Float | Strength of the pole. 0 turns it off |
| `<Arm>_High_Accuracy` | Bool | More solver iterations |
| `<Arm>_Tracking_Mode` | Bool | Start each solve from the current pose |

A move is: set the targets, set `Duration`, fire `Start`. The IK solves every frame, and `Start`
only controls the blend, so once a move has finished the hand follows target changes directly.
Firing `Start` again interrupts the current move.

`tools/RobotController-API.md` lists each input's default and description.

### Using RobotInput in code

```csharp
controller.SetInputBool(RobotInput.LeftArm_Tracking_Mode.ID(), true);
controller.SetInputVector3(RobotInput.LeftArm_Target_Position.ForArm(arm).ID(), pos); // 0 = left, 1 = right
RobotInputKind kind = RobotInput.Duration.Kind(); // Trigger, Bool, Float or Vector3
```

**When adding an input to the enum:**

- Add new members at the end only, because RobotAction saves the dropdown position.
- Keep each `RightArm_` member exactly 7 places after its `LeftArm_` twin. `ForArm` relies on it.
- Add the same name at the same position in the `Input` dropdown in `LD_RobotAction.cs`, and to
  `Kind()` if it isn't a Bool.
