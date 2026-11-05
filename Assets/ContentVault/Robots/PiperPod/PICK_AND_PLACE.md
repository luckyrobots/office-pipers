# Piper Pick-and-Place — Agent Reference

The Piper robot in this asset comes pre-wired with `MG_Piper.hmograph`. That graph is the **only** thing your script talks to: you push a target wrist pose + duration, the graph does the IK + min-jerk blend. Don't try to drive joints directly.

This doc is for an agent asked to "build a pick-and-place / stacking scene with Piper." Read the **Quick Start** to get something on screen. Read **The Tricks** before shipping — half the failures the original Pattern Stacking scene hit were not obvious from the API.

---

## 1. Quick Start — single pick-and-place in ~50 lines

```csharp
using Hazel;

namespace MyScene
{
    public class SimplePicker : Entity
    {
        public Vector3 PickPosition  = new Vector3(0.30f, 0.013f, -0.10f); // block centre
        public Vector3 PlacePosition = new Vector3(0.30f, 0.013f, +0.10f);
        public float   BlockYawRadians = 0.0f;

        // Motion-graph input IDs — names MUST match MG_Piper.hmograph exactly.
        private static readonly Identifier k_TargetPosition    = new Identifier("Target_Position");
        private static readonly Identifier k_TargetOrientation = new Identifier("Target_Orientation");
        private static readonly Identifier k_Duration          = new Identifier("Duration");
        private static readonly Identifier k_Start             = new Identifier("Start");

        private RobotControllerComponent? m_Robot;
        private MujocoActuator?           m_Gripper;
        private readonly TaskManager      m_Queue = new TaskManager();

        // Tunables — see "Numbers that work" further down.
        const float PreGraspLiftY = 0.25f;   // hover height above block centre
        const float GraspOffsetY  = 0.126f;  // wrist Y when fingers are on the block
        const float GripperOpen   = 255f;
        const float GripperClose  = 0f;
        const float GripperSettle = 1.0f;    // s, let fingers physically close
        const float TravelSpeed   = 0.5f;    // m/s for horizontal travel
        const float DescendSpeed  = 0.2f;    // m/s for vertical descents
        const float PlaceDescend  = 0.08f;   // m/s — slower for placing

        protected override void OnCreate()
        {
            m_Robot   = GetComponent<RobotControllerComponent>();
            var muj   = GetComponent<MujocoSceneComponent>();
            m_Gripper = muj?.GetActuator("gripper");
            if (m_Gripper is not null) { m_Gripper.IK = false; m_Gripper.Value = GripperOpen; }

            QueuePickAndPlace();
            m_Queue.Resume();
        }

        protected override void OnUpdate(float ts) => m_Queue.OnUpdate(ts);

        private void QueuePickAndPlace()
        {
            Vector3 rot = BuildGripperRotation(BlockYawRadians + Mathf.PIonTwo);

            Move(PickPosition  + new Vector3(0, PreGraspLiftY, 0), rot, TravelSpeed);   // hover above pick
            Move(PickPosition  + new Vector3(0, GraspOffsetY,  0), rot, DescendSpeed);  // descend
            Gripper(GripperClose, GripperSettle);                                       // grab
            Move(PickPosition  + new Vector3(0, PreGraspLiftY, 0), rot, DescendSpeed);  // lift
            Move(PlacePosition + new Vector3(0, PreGraspLiftY, 0), rot, TravelSpeed);   // travel
            Move(PlacePosition + new Vector3(0, GraspOffsetY,  0), rot, PlaceDescend);  // lower slow
            Gripper(GripperOpen, GripperSettle);                                        // release
            Move(PlacePosition + new Vector3(0, PreGraspLiftY, 0), rot, DescendSpeed);  // retreat
        }

        // ---- primitives ----
        private void Move(Vector3 pos, Vector3 rotRad, float speed)
        {
            float dist = Vector3.Distance(pos, m_LastPlanned);
            float dur  = Mathf.Max(0.15f, dist / Mathf.Max(speed, 0.001f));
            m_Queue.AddTask(() =>
            {
                m_Robot?.SetInputVector3(k_TargetPosition,    pos);
                m_Robot?.SetInputVector3(k_TargetOrientation, rotRad * Mathf.Rad2Deg);
                m_Robot?.SetInputFloat  (k_Duration,          dur);
                m_Robot?.SetInputTrigger(k_Start);
            }, dur);
            m_LastPlanned = pos;
        }
        private Vector3 m_LastPlanned = Vector3.Zero;

        private void Gripper(float value, float settleSec)
            => m_Queue.AddTask(() => { if (m_Gripper is not null) m_Gripper.Value = value; }, settleSec);

        // Wrist Euler for a top-down grasp at the given gripper yaw, kept
        // inside [π/2, 3π/2] — the IK branch that avoids reaching across body.
        public static Vector3 BuildGripperRotation(float yawRadians)
        {
            float y = Mathf.WrapToPi(yawRadians) + Mathf.PI; // → [0, 2π]
            if (y < Mathf.PIonTwo)            y += Mathf.PI;
            else if (y > 3f * Mathf.PIonTwo)  y -= Mathf.PI;
            return new Vector3(Mathf.PIonTwo, y, 0f);
        }
    }
}
```

That's the whole API surface for "drive the arm." Everything else in this doc is either a refinement or a way to avoid a failure mode.

---

## 2. The Motion Graph Contract

The robot entity has `RobotControllerComponent` + `MujocoSceneComponent`. The motion graph `MG_Piper.hmograph` is already attached. You interact with it by writing to **named graph inputs**:

| Input | Type | Unit | Required | Purpose |
|---|---|---|---|---|
| `Target_Position`    | Vec3  | metres                | yes | World-space wrist target |
| `Target_Orientation` | Vec3  | **degrees**, intrinsic XYZ Euler | yes | Wrist orientation. The arm controller stores radians internally then converts: `rotRad * Mathf.Rad2Deg` on the way in. |
| `Duration`           | Float | seconds               | yes | Time over which the graph blends from current pose → target. Min ≈ 0.15 s to avoid sub-frame moves. |
| `Start`              | Flow  | —                     | yes | Trigger. Fires one IK solve + one quintic blend. Call once per move. |
| `Solve_Orientation`  | Bool  | —                     | default true  | Whether IK matches wrist orientation. Leave at true. |
| `Pole_Target`        | Vec3  | metres                | optional      | Elbow pole-vector hint. Usually leave at (0,0,0). |
| `Pole_Weight`        | Float | —                     | optional      | Blend factor for pole hint, 0..1. |
| `High_Accuracy`      | Bool  | —                     | optional      | Enables higher-precision IK solve. |
| `Tracking_Mode`      | Bool  | —                     | optional      | Continuous tracking variant. |

**Internals (do NOT change in the graph):**
- LimbIK is wired with **End Effector = `link6`** (wrist) and **Chain Root = `base_link`**.
- Inside the graph: `Rest Pose` → `Stored Pose` (snapshot at `Start`) → `LimbIK` (solve) → `Joint States Blend` with `Min Jerk Curve` α → `Output`. One `Start` trigger = one solve + one min-jerk-eased blend.
- The graph does **not** loop or replan mid-move. To make a multi-segment path, queue multiple `Start` triggers back-to-back (see TaskManager pattern).

**Gripper:** the gripper is a MuJoCo actuator named `"gripper"`. Get it via `MujocoSceneComponent.GetActuator("gripper")`, then **set `actuator.IK = false`** once at OnCreate so direct value writes persist. Open value ≈ `255`, closed ≈ `0`.

**Finger bodies:** `link7` (left) and `link8` (right) — useful if you want to measure grasp pose: `(GetPosition(link7) + GetPosition(link8)) / 2` = gripper centre.

---

## 3. The Tricks (read before shipping)

These are the non-obvious things that took the Pattern Stacking scene from "tumbling arm + scattered blocks" to "clean stacks."

### 3.1 Always lift-then-translate-then-lower
A direct diagonal move from `home` to `block + GraspOffset` makes the wrist rotate at the same time it descends — the elbow swings wide and clips nearby blocks. Three pure-axis legs (vertical lift, horizontal travel, vertical drop) is dramatically cleaner because each segment's joint-space delta is small:

```csharp
float safeY = Mathf.Max(currentY, targetY + 0.10f);  // 100 mm above target
Move(new Vector3(currentX, safeY, currentZ), rot, 0.15f);  // 1. lift in place
Move(new Vector3(targetX,  safeY, targetZ),  rot, 0.5f);   // 2. lateral at safe altitude
Move(new Vector3(targetX,  targetY, targetZ), rot, 0.2f);  // 3. drop straight down
```

Use this for **every** approach to a block — pickup AND placement.

### 3.2 Gripper wrist Euler — fixed pitch, restricted yaw
A two-finger gripper is symmetric about its yaw axis (yaw and yaw+π are the same physical grasp). The IK solver will happily pick either, and switching branches between moves makes the arm tumble. Force the canonical half-circle `[π/2, 3π/2]`:

```csharp
public static Vector3 BuildGripperRotation(float yawRadians)
{
    float y = Mathf.WrapToPi(yawRadians) + Mathf.PI;
    if (y < Mathf.PIonTwo)            y += Mathf.PI;
    else if (y > 3f * Mathf.PIonTwo)  y -= Mathf.PI;
    return new Vector3(Mathf.PIonTwo, y, 0f);  // pitch=π/2 (top-down), roll=0
}
```

Add a `GripperYawOffset = π/2` to the block's yaw before passing it in — that aligns the fingers' close-axis with the block's short edge:

```csharp
Vector3 rot = BuildGripperRotation(blockYaw + Mathf.PIonTwo);
```

### 3.3 Read block yaw from the quaternion, not Euler.Y
A block lying in "flipped" Euler form (pitch/roll at ±π) has the same physical orientation as `(0, ψ, 0)` but `WorldRotation.Y` returns the wrong branch ~half the time. Project the local +Z axis through the world quaternion and `atan2` it on the ground plane:

```csharp
private static float GetVisualYaw(Entity e)
{
    Vector3 fwd = e.Transform.WorldRotationQuat * new Vector3(0f, 0f, 1f);
    return Mathf.Atan2(fwd.X, fwd.Z);
}
```

This single fix removed the majority of "block placed at wrong angle" failures.

### 3.4 Hover-drop when there's a same-layer neighbour ALREADY PLACED
On any block in a multi-block layer that has **at least one already-placed sibling in the same layer** (Brick / Jenga / Chimney / Ladder), descending all the way to `placeY + GraspOffsetY` and then opening the gripper pushes the fingers outward across the neighbour's top face, shoving the neighbour sideways. Release from `1.5 × blockHeight` above the slot so the fingers open in empty air:

```csharp
// CORRECT — order-independent test
bool hasPlacedSibling = placedInThisLayerSoFar > 0;
float hover = hasPlacedSibling ? BlockSize.Y * 1.5f : 0.0f;
Vector3 release = placePos + new Vector3(0, GraspOffsetY + hover, 0);
```

The rule is **placement-order-based, NOT column-based**. First block placed in a layer → no hover. Every subsequent block in that layer → hover. Holds regardless of which column you place first.

> ⚠ **Common bug:** writing `hover = (slot == centreColumn) ? 0 : 1.5*blockH;` works ONLY if the centre is placed first. If your loop iterates columns 0, 1, 2 in left-centre-right order, the centre is placed second with a placed left-neighbour but the column-test still says "no hover" — and the gripper-open knocks the left block out. Always test "has a sibling been placed in this layer," never "am I the centre."

The block tumbles a little in free-fall but the alternative is knocking a previously-placed neighbour over, which is strictly worse.

### 3.5 Slower / higher / longer for upper layers
Drops onto layer 2+ transmit force through the support and topple the layer below. Compound a per-layer multiplier:

```csharp
float layerHover     = baseHover    * Mathf.Pow(1.5f,  layer);  // higher release each layer up
float layerDescend   = PlaceDescend * Mathf.Pow(0.7f,  layer);  // slower descent
float layerSettleSec = 0.3f         * Mathf.Pow(1.6f,  layer);  // longer dwell before next pickup
```

Layer 0 = `1.0×`, layer 1 = `1.5× / 0.7× / 1.6×`, layer 2 = `2.25× / 0.49× / 2.56×`, etc.

### 3.6 Anchor placements on live block poses, not theoretical positions
After placing 3 blocks, the next block's "where does it go" is **the actual measured centroid of the layer below**, not the precomputed slot position. Blocks settle, drift, get nudged — chase the real pose:

```csharp
Vector3 supportCentre = Vector3.Zero;
int n = 0;
foreach (var placed in layerBelow) {
    supportCentre += placed.Transform.WorldTranslation;
    n++;
}
supportCentre /= n;
Vector3 placePos = supportCentre + slotOffsetFromCentre;
```

Optional refinement: blend with the theoretical centre to apply a corrective torque on a drifted support (Pattern Stacking uses `bias = 0.75`):

```csharp
Vector3 basePos = liveCentroid + 0.75f * (intendedCentroid - liveCentroid);
```

### 3.7 Verify the grab actually happened
After the pickup retreat, check the block actually rose:

```csharp
float actualLift = blockNow.Y - blockBeforePickup.Y;
float requiredLift = (PreGraspLiftY - GraspOffsetY) * 0.5f;  // 50 % of net retreat
if (actualLift < requiredLift) { /* retry pickup or abort */ }
```

`PickupRetryLimit = 2` is a good default — past that, the block is probably knocked over and on its fat side.

### 3.8 Don't try to pick up a tipped-over block
The fingers can only grasp upright blocks. Before each pickup, check that the local +Y axis still points up:

```csharp
Vector3 localUp = block.Transform.WorldRotationQuat * Vector3.Up;
bool onFatSide = Mathf.Abs(localUp.Y) < 0.85f;
```

If `true`, abort or skip that block.

### 3.9 Gentle gripper open at placement
Snapping the gripper from `0` → `255` instantly flicks the fingers outward against the block's faces and shoves it sideways. Ramp the open in 4–5 linear steps over ~0.15 s:

```csharp
for (int i = 1; i <= 5; i++) {
    float v = GripperClose + (GripperOpen - GripperClose) * (i / 5.0f);
    m_Queue.AddTask(() => m_Gripper!.Value = v, 0.15f / 5);
}
```

Then dwell ~0.3 s (`PlaceSettle`) before retreat so the block settles with near-zero residual velocity.

### 3.10 Floor the duration by angular speed AND distance
A short translation that involves a large wrist twist will rip the block out of the fingers if duration is set by translation alone. Compute both and take the max:

```csharp
float linDur = distance / TravelSpeed;
float rotDur = Mathf.Abs(Mathf.WrapToPi(targetYaw - currentYaw)) / 1.5f;  // 1.5 rad/s ≈ 86°/s
float duration = Mathf.Max(0.15f, Mathf.Max(linDur, rotDur));
```

---

## 4. Reusable Arm Controller Pattern

For anything beyond a single-shot demo, copy this skeleton instead of inlining moves. (This is essentially the public surface of `StackingArmController` — strip whatever you don't need.)

```csharp
public class ArmController : Entity
{
    public string GripperActuatorName = "gripper";
    public Vector3 Home              = new Vector3(0.42f, 0.28f, 0.0f);
    public Vector3 HomeRotRadians    = new Vector3(-Mathf.PIonTwo, 0f, -1.705f);
    public Vector3 GraspOffset       = new Vector3(0f, 0.126f, 0f);
    public Vector3 PreGraspLift      = new Vector3(0f, 0.25f,  0f);
    public float   SafeApproachLift  = 0.10f;
    public float   TravelSpeed       = 0.5f;
    public float   DescendSpeed      = 0.2f;
    public float   PlaceDescendSpeed = 0.08f;
    public float   MaxAngularSpeed   = 1.5f;
    public float   MinMoveDuration   = 0.15f;
    public float   AlignDwell        = 0.2f;
    public float   GripperOpenValue  = 255f;
    public float   GripperCloseValue = 0f;
    public float   GripperSettle     = 1.0f;
    public float   GripperYawOffset  = Mathf.PIonTwo;

    private RobotControllerComponent? m_Robot;
    private MujocoActuator?           m_Gripper;
    private readonly TaskManager      m_Queue = new TaskManager();
    private Vector3 m_PlannedPos, m_PlannedRot;

    private static readonly Identifier k_Pos = new Identifier("Target_Position");
    private static readonly Identifier k_Rot = new Identifier("Target_Orientation");
    private static readonly Identifier k_Dur = new Identifier("Duration");
    private static readonly Identifier k_Go  = new Identifier("Start");

    public bool IsBusy => m_Queue.IsRunning;
    public void Continue() => m_Queue.Resume();
    public void ClearQueue() { m_Queue.Clear(); if (m_Gripper is not null) m_Gripper.Value = GripperOpenValue; m_PlannedPos = Home; m_PlannedRot = HomeRotRadians; }
    public Vector3 PlannedEndPosition => m_PlannedPos;

    public void EnqueueMoveAtSpeed(Vector3 pos, Vector3 rotRad, float speed)
    {
        float dist   = Vector3.Distance(pos, m_PlannedPos);
        float rotMag = Mathf.Abs(Mathf.WrapToPi(rotRad.Y - m_PlannedRot.Y));
        float dur    = Mathf.Max(MinMoveDuration, Mathf.Max(dist / speed, rotMag / MaxAngularSpeed));
        m_Queue.AddTask(() => Send(pos, rotRad, dur), dur);
        m_PlannedPos = pos; m_PlannedRot = rotRad;
    }

    public void EnqueueOpenGripper (float? s = null) => m_Queue.AddTask(() => { if (m_Gripper is not null) m_Gripper.Value = GripperOpenValue;  }, s ?? GripperSettle);
    public void EnqueueCloseGripper(float? s = null) => m_Queue.AddTask(() => { if (m_Gripper is not null) m_Gripper.Value = GripperCloseValue; }, s ?? GripperSettle);
    public void EnqueueWait  (float s)       => m_Queue.AddTask(null, s);
    public void EnqueueAction(System.Action a, float dwell = 0f) => m_Queue.AddTask(a, dwell);
    public void EnqueueGoHome()              => EnqueueMoveAtSpeed(Home, HomeRotRadians, TravelSpeed);

    // Routed approach: lift → travel → drop, each on a pure axis.
    public void EnqueueRoutedApproach(Vector3 target, Vector3 rotRad)
    {
        float safeY     = Mathf.Max(m_PlannedPos.Y, target.Y + SafeApproachLift);
        Vector3 liftHere   = new Vector3(m_PlannedPos.X, safeY, m_PlannedPos.Z);
        Vector3 overTarget = new Vector3(target.X,       safeY, target.Z);
        EnqueueMoveAtSpeed(liftHere,   rotRad, 0.15f);
        EnqueueMoveAtSpeed(overTarget, rotRad, TravelSpeed);
        EnqueueMoveAtSpeed(target,     rotRad, DescendSpeed);
    }

    protected override void OnCreate()
    {
        m_Robot   = GetComponent<RobotControllerComponent>();
        var muj   = GetComponent<MujocoSceneComponent>();
        m_Gripper = muj?.GetActuator(GripperActuatorName);
        if (m_Gripper is not null) { m_Gripper.IK = false; m_Gripper.Value = GripperOpenValue; }
        m_PlannedPos = Home; m_PlannedRot = HomeRotRadians;
    }
    protected override void OnUpdate(float ts) => m_Queue.OnUpdate(ts);

    private void Send(Vector3 pos, Vector3 rotRad, float dur)
    {
        if (m_Robot is null) return;
        m_Robot.SetInputVector3(k_Pos, pos);
        m_Robot.SetInputVector3(k_Rot, rotRad * Mathf.Rad2Deg);
        m_Robot.SetInputFloat  (k_Dur, dur);
        m_Robot.SetInputTrigger(k_Go);
    }

    public static Vector3 BuildGripperRotation(float yawRadians)
    {
        float y = Mathf.WrapToPi(yawRadians) + Mathf.PI;
        if (y < Mathf.PIonTwo)           y += Mathf.PI;
        else if (y > 3f * Mathf.PIonTwo) y -= Mathf.PI;
        return new Vector3(Mathf.PIonTwo, y, 0f);
    }
}
```

Composing a pickup with this controller is then ~10 lines:

```csharp
Vector3 rot = ArmController.BuildGripperRotation(GetVisualYaw(block) + arm.GripperYawOffset);
Vector3 pre = block.Transform.WorldTranslation + arm.PreGraspLift;
Vector3 hit = block.Transform.WorldTranslation + arm.GraspOffset;
arm.EnqueueOpenGripper(0.15f);
arm.EnqueueRoutedApproach(pre, rot);
arm.EnqueueWait(arm.AlignDwell);
arm.EnqueueMoveAtSpeed(hit, rot, arm.DescendSpeed);
arm.EnqueueCloseGripper();
arm.EnqueueMoveAtSpeed(pre, rot, arm.DescendSpeed);
arm.Continue();
```

---

## 5. Pattern Library

If the user asks for a specific stacking shape, generate slots in pattern-local space (Y up, target at origin) then add the target's world position + yaw. **Place layers bottom-up, and within each layer place the middle / centre-most block first** so subsequent blocks can anchor on it.

Common: `LayerY(layer, h) = h * (layer + 0.5f)` — block centre at layer N is at `(N + 0.5)·blockHeight` above the support surface.

### Tower (1 / layer)
```csharp
for (int i = 0; i < count; i++)
    slots.Add((local: new Vector3(0, LayerY(i, h), 0), yaw: 0));
```
Centre over target, one block per layer. **Cap at ~4 layers** for Piper's reach.

### Jenga (3 / layer, alternating 90°)
```csharp
float spacing = blockSize.X;        // short edge → blocks are side-by-side along long axis
int perLayer  = 3;
for (int s = 0; s < count; s++) {
    int layer = s / perLayer, idx = s % perLayer;
    float off = idx switch { 0 => 0f, 1 => -spacing, _ => +spacing };  // middle, left, right
    bool rotated = (layer & 1) == 1;
    Vector3 local = rotated ? new Vector3(0, LayerY(layer, h), off)
                            : new Vector3(off, LayerY(layer, h), 0);
    float yaw = rotated ? Mathf.PIonTwo : 0f;
    slots.Add((local, yaw));
}
```
Order per layer: **middle first**, then left, then right.

### Brick (3-2-3-2 running bond)
```csharp
float L = blockSize.Z;                                     // long edge
const float gap = 0.008f;                                  // CRITICAL — see below
float threeBrickSide = L + gap;                            // outer centres of a 3-brick layer
float wideOffset     = L * (0.75f + gap / L);              // 2-brick layer half-spacing
// layer parity: even → 3-brick (z = 0, -threeBrickSide, +threeBrickSide), odd → 2-brick (z = ±wideOffset)
```
**The 8 mm side gap** between bricks in the 3-brick layers is load-bearing. Without it, the descending outer block rides up onto the centre block's top face during placement.

### Chimney (4 / layer hollow ring, rotates 90°)
```csharp
float r = blockSize.Z * 0.5f;
// layer parity: rotate the ring 90° each layer
// idx 0..3: +Z, -Z, +X, -X (yaws: π/2, π/2, 0, 0)
```

### LadderChimney (2 / layer, alternating rungs)
```csharp
float r = blockSize.Z * 0.5f;
// even layer: rung along Z → two blocks at z = +r and z = -r, yaw = π/2
// odd  layer: rung along X → two blocks at x = +r and x = -r, yaw = 0
```

### Picking a pattern from user intent
- "Stack" / "tower" / no shape mentioned → **Tower**
- "Jenga" → **Jenga**
- "Wall" / "brick" → **Brick**
- "Hollow" / "chimney" → **Chimney**
- "Ladder" → **LadderChimney**

For "I just want one block moved" — skip the pattern library entirely, just compose a single pickup + single placement.

---

## 6. Numbers That Work

Defaults from the calibrated Pattern Stacking scene. Reasonable starting point; the user can tune later.

| Constant | Value | Notes |
|---|---|---|
| `Home` position | `(0.42, 0.28, 0.00)` m | Wrist parked above + forward of base. |
| `HomeRotationRadians` | `(-π/2, 0, -1.705)` | Wrist down, neutral yaw. |
| `GraspOffset.Y` | `0.126` m | Wrist target = block centre + this. Specific to Piper's link6 → finger-tip distance. |
| `PreGraspLift.Y` | `0.25` m | Hover height above block centre before/after grasp. |
| `SafeApproachLift` | `0.10` m | Extra altitude for the lateral leg of a routed approach. |
| `TravelSpeed` | `0.5` m/s | Long horizontal moves. |
| `DescendSpeed` | `0.2` m/s | Short vertical moves (descents, retreats). |
| `PlaceDescendSpeed` | `0.08` m/s | Slow, gentle drop onto a placement. |
| `SafeApproachLiftSpeed` | `0.15` m/s | First vertical leg of routed approach (while parked inside spawn area). |
| `MaxAngularSpeed` | `1.5` rad/s (≈ 86°/s) | Twist faster than this and blocks slip. |
| `MinMoveDuration` | `0.15` s | Floor on any single move's duration. |
| `AlignDwell` | `0.2` s | Settle dwell after the diagonal approach finishes. |
| `GripperOpenValue` | `255` | MuJoCo actuator command. |
| `GripperClosedValue` | `0` | |
| `GripperSettle` | `1.0` s | After open/close, before next motion. |
| `GripperOpenRamp` | `0.15` s in 5 steps | For gentle placement release. |
| `PlaceSettle` | `0.3` s | After descent, before opening fingers. |
| `GripperYawOffset` | `π/2` | Add to block yaw → aligns finger close-axis with short edge. Calibrated — don't change. |
| Piper finger max opening | ≈ `0.020` m | **The block's X (short horizontal) MUST be ≤ this or the fingers can't close around it.** |
| `ReachRadius` (theoretical max) | `0.5` m | Hard outer wall (XZ from base). |
| **`ReachRadius` (solver-reliable)** | **`0.35` m** | Use this when picking a stack centre. Past 0.35 m, top-layer IK starts to fail. |
| Max wrist Y (solver-reliable) | `0.40` m | Limits the top-layer pre-grasp height for tall stacks. |
| Recommended stack centre, 3-layer Jenga | `(0.24, 0, 0)` | Calibrated working position. Keep `stackCentre.Z` near 0 — off-axis reach is the most limited. |
| Block default size | **`(0.015, 0.025, 0.075)` m** | **X = SHORT horizontal (≤ 20 mm, gripper closes across this), Y = HEIGHT, Z = LONG horizontal. DO NOT swap these — the `+π/2` rotation formula assumes Z is the long axis. Putting 0.025 in X (a common mistake) makes the block too wide for the gripper AND the rotation rotates to the wrong axis.** |

---

## 7. Failure Modes Cheat Sheet

| Symptom | Cause | Fix |
|---|---|---|
| `input with id N does not exist` log | Identifier string doesn't match a graph input name. | Use exact names from §2 — `Target_Position`, `Target_Orientation`, `Duration`, `Start`. Case + underscores matter. |
| Position values appear as orientation (and vice versa) | The Input node labels in `MG_Piper.hmograph` got swapped during a rename. | Open the motion graph; the Input node wired to LimbIK's **Target Position** pin must be labelled `Target_Position`. Same for orientation. |
| Arm tumbles / "reaches across body" | IK branch flips between two valid solutions because gripper yaw isn't canonicalised. | Always pass wrist Euler through `BuildGripperRotation` (§3.2). |
| Block placed at ~90° off intended yaw | Reading `WorldRotation.Y` on a flipped-Euler block. | Use the quaternion-based `GetVisualYaw` (§3.3). |
| Same-layer neighbour knocked aside on placement | Fingers descend into neighbour's top face. | Hover-drop (§3.4). |
| Layer-2 placement collapses layer 0/1 | Drop impulse transmits through the stack. | Layer cascade scaling (§3.5). |
| Block doesn't move with the gripper after close | `m_Gripper.IK` left at default (true) — IK keeps re-opening it. | Set `m_Gripper.IK = false` once at OnCreate. |
| First move runs but subsequent moves don't | `m_Queue.Resume()` wasn't called after enqueuing the new batch. | Call `Continue()` (or `m_Queue.Resume()`) after every batch of enqueues. |
| "Sub-frame" duration warning / instant jump | Distance-derived duration is below frame time. | Floor `Mathf.Max(MinMoveDuration, …)` on every duration calc. |
| Block consistently lands a few mm off in the same direction | Systematic release-offset bias (gripper local frame). | Either add `PlaceXZNudge` (constant world bias), or for varying target rotations add an EWMA in the gripper's local frame (rotate by `-yaw` before averaging, by `+yaw` to apply). |

---

## 8. Where to Look in This Repo

- `MG_Piper.hmograph` — motion graph. Inputs / pins listed in §2.
- `piper.xml` / `scene.xml` — MuJoCo definitions (joint names, actuator names, finger body names `link7` / `link8`).
- `assets/` — Piper meshes (`base_link`, `link1`–`link8`).

If the user wants a richer reference implementation than the snippets above (full diagnostic logging, anchor-chain placements, grip-offset compensation, auto-calibrated release bias), point them at the **Piper Pattern Stacking** example scene in the engine's ContentVault — that's where every trick in §3 was originally validated.
