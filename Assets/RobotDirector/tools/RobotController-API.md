# Robot controller input reference

This lists all exposed input names in this project's OpenArm Pedestal and Panda
controller graphs, including the names used by
[CurveFollower.cs](Assets/Scripts/Client/Source/CurveFollower.cs).
Types, defaults and descriptions come from the graph assets.

These strings are **graph-defined input names**, not a universal list built into
`RobotControllerComponent`. Use the names belonging to the graph assigned to the
robot. Renaming or adding an exposed input in a graph changes its interface.

## How an Identifier is used

```csharp
Identifier inputID = new Identifier("LeftArm_Target_Position");
Vector3 target = new Vector3(0.3f, 0.5f, 0.2f);

m_Controller.SetInputVector3(inputID, target);
```

- `inputID` selects the named input on the controller.
- `target` supplies its new value.
- `SetInputVector3` is the setter for a `Vector3` input.
- Creating an `Identifier` alone does not create or change a graph input.
- Copy the exact spelling and underscores from the tables below.

## Methods for named inputs

These public signatures were inspected in the installed Lucky Engine 2026.3
`Hazel-ScriptCore` library.

| Input type | Write | Read |
|---|---|---|
| `bool` | `SetInputBool(Identifier inputID, bool value)` | `GetInputBool(Identifier inputID)` |
| `float` | `SetInputFloat(Identifier inputID, float value)` | `GetInputFloat(Identifier inputID)` |
| `int` | `SetInputInt(Identifier inputID, int value)` | `GetInputInt(Identifier inputID)` |
| `Vector3` | `SetInputVector3(Identifier inputID, Vector3 value)` | `GetInputVector3(Identifier inputID)` |
| Trigger pulse | `SetInputTrigger(Identifier inputID)` | No `GetInputTrigger` method is exposed. |

The graphs below do not expose any integer inputs. Their `Float3` graph values
correspond to C# `Vector3`. Although `Start` is a boolean node in each graph, its
description calls for a one-update pulse, so the script uses `SetInputTrigger`.

## OpenArm Pedestal: all 16 input names

Source: [MG_OpenArmPedestal.hpodgraph](Assets/ContentVault/Robots/OpenArmPedestalPod/MG_OpenArmPedestal.hpodgraph).

### Shared motion controls

| Identifier string | Type / setter | Graph default | Purpose |
|---|---|---|---|
| `Start` | Pulse / `SetInputTrigger` | `false` | Capture the starting pose and restart the motion timer. Fire after setting targets and duration. This is shared by both arms, not a separate start input per arm. |
| `Duration` | `float` / `SetInputFloat` | `3` seconds | Motion duration in seconds. The motion timer captures the duration when restarted; set this before firing `Start`. |

### Left arm

| Identifier string | Type / setter | Graph default | Purpose |
|---|---|---|---|
| `LeftArm_Target_Position` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | Target position in world space, in metres. |
| `LeftArm_Target_Orientation` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | Target orientation as world-space Euler angles in degrees, not a quaternion. Used when orientation solving is enabled. |
| `LeftArm_Solve_Orientation` | `bool` / `SetInputBool` | `true` | Include the target orientation in the inverse-kinematics solve. The curve follower disables this when its `SolveOrientation` setting is false. |
| `LeftArm_Pole_Target` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | World-space pole position used to guide the limb bend. |
| `LeftArm_Pole_Weight` | `float` / `SetInputFloat` | `0` | Influence of the pole target on the limb bend. Zero disables the pole bias. |
| `LeftArm_High_Accuracy` | `bool` / `SetInputBool` | `false` | Select the solver's higher-accuracy mode. The graph does not specify numerical accuracy or performance guarantees. |
| `LeftArm_Tracking_Mode` | `bool` / `SetInputBool` | `false` | Select the solver's tracking mode for a changing target. The curve follower enables this for the arm following the curve. |

### Right arm

| Identifier string | Type / setter | Graph default | Purpose |
|---|---|---|---|
| `RightArm_Target_Position` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | Target position in world space, in metres. |
| `RightArm_Target_Orientation` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | Target orientation as world-space Euler angles in degrees, not a quaternion. Used when orientation solving is enabled. |
| `RightArm_Solve_Orientation` | `bool` / `SetInputBool` | `true` | Include the target orientation in the inverse-kinematics solve. |
| `RightArm_Pole_Target` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | World-space pole position used to guide the limb bend. |
| `RightArm_Pole_Weight` | `float` / `SetInputFloat` | `0` | Influence of the pole target on the limb bend. Zero disables the pole bias. |
| `RightArm_High_Accuracy` | `bool` / `SetInputBool` | `false` | Select the solver's higher-accuracy mode. The graph does not specify numerical accuracy or performance guarantees. |
| `RightArm_Tracking_Mode` | `bool` / `SetInputBool` | `false` | Select the solver's tracking mode for a changing target. |

## Panda: all 9 input names

Source: [MG_Panda.hpodgraph](Assets/ContentVault/Robots/PandaPod/MG_Panda.hpodgraph).
Panda's input names do not have a `LeftArm_` or `RightArm_` prefix.

| Identifier string | Type / setter | Graph default | Purpose |
|---|---|---|---|
| `Start` | Pulse / `SetInputTrigger` | `false` | Pulse for one update after setting the target and duration to capture the starting pose and restart the motion timer. |
| `Duration` | `float` / `SetInputFloat` | `3` seconds | Motion duration in seconds. |
| `Target_Position` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | Target position in world space, in metres. |
| `Target_Orientation` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | Target orientation as world-space Euler angles in degrees. |
| `Solve_Orientation` | `bool` / `SetInputBool` | `true` | Include the target orientation in the inverse-kinematics solve. |
| `Pole_Target` | `Vector3` / `SetInputVector3` | `(0, 0, 0)` | World-space pole position used to guide the limb bend. |
| `Pole_Weight` | `float` / `SetInputFloat` | `0` | Influence of the pole target on the limb bend. Zero disables the pole bias. |
| `High_Accuracy` | `bool` / `SetInputBool` | `false` | Select the solver's higher-accuracy mode. |
| `Tracking_Mode` | `bool` / `SetInputBool` | `false` | Select the solver's tracking mode for a changing target. |

## How CurveFollower uses these inputs

1. Selects `LeftArm` or `RightArm` from its `Arm` setting.
2. Sets the other arm's target to its current grasp-site world position and
   disables that arm's orientation solve, so it does not chase the default origin
   when the shared `Start` pulse fires. This step requires a MuJoCo scene component.
3. Enables tracking mode for the selected arm.
4. Sets its orientation-solving flag, target orientation and first path position.
5. Sets `Duration` to `ApproachTime`, then fires `Start`.
6. After the approach time, updates the selected arm's target position each frame.

For example:

```csharp
string side = "LeftArm";
Identifier targetID = new Identifier($"{side}_Target_Position");

m_Controller.SetInputBool(new Identifier($"{side}_Tracking_Mode"), true);
m_Controller.SetInputVector3(targetID, PointAt(path, m_Distance));
```

Here, `$"{side}_Target_Position"` evaluates to `"LeftArm_Target_Position"`.
The name selects the graph input; `PointAt(...)` calculates the value sent to it.

Graph defaults in the tables are the values saved in the assets, not necessarily
the current runtime values after a script has changed them.

## Names that are not public controller inputs

Internal graph parameters such as `node_16_Target`, joint/site names such as
`left_grasp_site`, and state attributes such as `_state_time` are not exposed
controller input names. Do not treat them as additional `SetInput...` identifiers.
The separate [CurvePath graph](Assets/Graphs/CurvePath.hpodgraph) has no exposed
inputs and is not included in the robot input tables.
