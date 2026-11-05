# Wood-filler pick-and-place task

This project task runs in **simulation Play mode**. It never connects to the hardware server or sends commands to a physical robot. The Pipers plugin's live hardware feedback currently operates in Edit mode; it is a separate workflow.

## Scene and scripts

- `Assets/ContentVault/Robots/PiperPod/piper_tube_task.xml` is the shared MuJoCo world: active Piper/Pika, free tube, tabletop, and fixed collision geometry for the other three arms. Separate MuJoCo instances cannot make grasp contacts with each other.
- `WoodFillerTask.cs` owns prepare/start/finish and outcome checks.
- `WoodFillerLayout.cs` samples reproducible start positions from a seed and preparation ordinal.
- `WoodFillerDemonstrator.cs` explicitly runs a slow simulated pick-and-place through the existing Piper motion graph. It performs no startup homing or gripper commands.
- `WoodFillerTrajectory.cs` captures measured joint positions and publishes replay artifacts.

Assign the task's `PhysicsWorld` to the combined world, `ActiveRobotBase` to its Piper root, the three other base anchors, and an optional static-mesh `StartMarkerTemplate`. Set `PositionActuatorModelSource` to the exact `piper_tube_task.xml` source path. Attach the demonstrator to the combined world and assign its `TaskEntity` to the task host. The existing graph end-effector is `grasp_site`; the grasp pad reference is `task_grasp_site`. Both position and orientation offsets are measured from the live model. LuckyIK expects the site orientation, not the `link6` wrist orientation; the fixed Pika mount rotates those frames by 90 degrees.

Before preparation, a worker checks the source SHA-256 (`63a5ba5b6be49eeeb6d216594714255d61a8d50b41d4ef799fb3b8f20e519163`) and its seven ordered `<position>` actuator tags. A changed source is rejected until deliberately requalified. Runtime checks verify actuator names, joint transmissions, ranges, and absence of policy ownership. Public ScriptCore cannot introspect the complete live actuator law; the export labels the authored qualification as provenance rather than an independent runtime measurement.

The Minwax tube is 193 mm tall with a measured round reference diameter of 43 mm, standing cap-down. It is not a constant-diameter cylinder: the actual scan preserves an upper flattened crimp about 77 mm wide. `TubeDiameter` and the exported `diameterMm` identify the round reference measurement. The visual scan, four convex contact meshes and their provenance/qualification records are in `MINWAX-SCAN-PROVENANCE.json` and `MINWAX-CONTACT-QUALIFICATION.json` beside the task XML. Its native body uses **local +Z up** with its origin at mid-height. On this scene's tabletop at world Y = 0.75 m, its resting origin is approximately Y = 0.8465 m. Mass is an estimate, 0.18 kg; weigh it before treating dynamics as calibrated.

The goal is the existing desk cross, **LEFT 903 mm / BACK 350 mm**, not the geometric centre of the 1400 × 700 mm desk. LEFT/BACK use the confirmed desk edges and do not change with the user's seating position.

## Run an episode

1. Open `Assets/Scenes/TubePickPlace.hscene`, enter Play, and select **Wood filler task - episode controls**. Keep `Workflow = SimulationOnly`. To record native virtual media, enable `RecordVirtualScene`, configure the Observer and sign in when requested. Leave that option off for standalone expert JSON export.
2. Click **Prepare random start**. The task samples a start, resets only the free tube in its owning world, then checks that it has settled upright. No native recording episode is open during this reset.
3. Click **Begin simulation-only episode**. A unique start manifest is saved before opening capture. The actual initial pose and qpos are captured again at the begin sync boundary. The temporary start marker is hidden before recording.
4. Select **Piper 1 + tube - shared task physics** and click **Run simulated demonstration**, or provide your own simulated control. Keep `DurationScale = 1.2` for the qualified expert. The driver ramps the opening to 80 mm, approaches, closes toward 36 mm around the tube, checks that it really lifted, carries it to the cross, releases, and withdraws. It fails rather than pretending that an unreachable target or missed grasp succeeded.
5. The driver requests success only after the task sees a prior lift and an upright, motionless tube at the cross. For manual demonstrations, use **Tube released at cross — finish successfully** after releasing it. Use **Finish as unsuccessful** for a failed attempt. Wait for export to finish before preparing another start.

The qualified starting region is deliberately small: **LEFT 950–980 mm / BACK 160–220 mm**, with a 120 mm nominal lift. Offline contact checks cover the starting positions and tube yaws listed in the contact qualification record. The native run below verifies one generated episode through the actual editor motion graph; it does not establish reachability over the whole desk. Expanding the region or changing the tube, tool, approach, physics, or robot placement requires new checks. The upright side-grasp keeps the approach radial to the robot base; a perfect top-down pose was not reachable throughout this region within the physical joint limits.

Native Play applies existing gripper tuning overrides that differ from authored XML gains. Both the model assumptions and the active native tuning matter when interpreting simulated dynamics. Successful simulated contact is not proof of matching real-world forces or grasp reliability.

The qualified task model uses MuJoCo `noslip_iterations="3"` to suppress slow soft-contact creep. The recorded native control path reproduced the earlier slip; the revised solver setting passed the recorded slow lift, extra hold and the qualification grid without increasing the 36 mm closing target, friction or motor force. See the qualification report for contact-force and friction-cone checks. This is a solver qualification, not a calibration of real gripping forces. MuJoCo documents NoSlip as residual-drift suppression with added computation and limitations for inverse dynamics; this dataset records position targets rather than inverse-dynamics torques. [MuJoCo contact guidance](https://mujoco.readthedocs.io/en/3.17.0/modeling.html#preventing-slip)

The demonstrator ramps gripper position commands over each grip step instead of jumping directly to the endpoint. It reads the current native command at the start of the ramp. `DurationScale = 1.2` extends graph movement durations; set this field explicitly in older saved scenes when loading the updated script. The exported channels always contain the actual applied commands, so physical replay can still enforce its own verified speed and acceleration limits.

## Verified native episode

On 2026-11-03, the native Lucky Engine demonstration completed the lift, carry, placement, release and withdrawal with the actual Minwax scan and qualified `63a5…9163` model. Artifact `6687789ff6a247af92ce741ac9d22291` in task session `d4297bbb70d94f6083ca625741ef42e3` exported `outcome = success` after the simulated placement checks.

The expert JSON contains **2,826 samples over 56.5 seconds**, at 20 ms intervals with no missing ticks. Native recording independently finalized successfully with **2,825 data rows and four 50 fps MKV streams**; its frame rows align with the JSON to less than `3.1e-11` seconds. The JSON's extra row is the terminal endpoint. Joint positions and position commands stayed within the reported joint limits; maximum sampled tracking error was 0.881°. The gripper command ramp's largest recorded step was 1.761 mm, replacing the earlier instantaneous full closure.

This run began at **LEFT 972.2084 mm / BACK 213.2890 mm / yaw 179.2037°**. The recorded first measured joint pose, J1–J6, was approximately **[−0.000054°, +0.021024°, −0.168152°, +0.000673°, −0.022694°, +0.00000024°]**, with **44.04438 mm** jaw opening. Its first commanded joint reference was [0°, 0°, 0°, 0°, 0°, 0°]. These are different channels: use the measured first state to verify physical alignment, and the command references to replay. Do not infer an implicit homing move from the near-zero initial pose. Subsequent episodes start from their own captured state; preparation resets the tube, not the robot.

The native simulation succeeded, but its original command timing is **not qualified for direct physical replay**. Peak sampled joint-command speed was 15.418°/s, below the reported 17.189°/s limit. Reference acceleration corners reached about 653.525°/s², above the reported 28.648°/s² limit. An acceleration-aware physical planner must retime the trajectory; uniform scaling alone needs at least approximately 4.777× for those sampled bounds. Gripper timing must also satisfy the selected hardware profile. This evidence does not claim a physical replay or calibrated real gripping forces.

## Recording and exports

`ExportTrajectory` writes to `ExportDirectory`, defaulting to `%LOCALAPPDATA%/LuckyRobots/OfficePipers/TubeEpisodes/<task-session-id>/`. Each attempt has a unique artifact ID:

- `<id>.start.json`: durable preparation snapshot, written before the episode opens.
- `<id>.json`: final version 2 trajectory and the actual begin-state snapshot. Samples contain elapsed simulation seconds, sync index, six measured joint angles, measured Pika jaw opening, and separate **actual native position-actuator command targets**. Joint channels are degrees; opening channels are millimetres. Commands come from `mjData.ctrl`, not from observed joint positions. Motor torques are not exported.

`jointDegrees` and `gripperOpeningMm` are observations. `commandedJointDegrees` and `commandedGripperOpeningMm` are the active position-control references. For example, while grasping this 43 mm tube, the actual opening can remain near 43 mm while the controller requests 36 mm to maintain contact. A physical replay must retain that distinction. The model's allowed maximum opening is 90 mm; its separate geometric zero-slide reference gap is approximately 94.044382 mm.

Commands and observations are read after physics on every robot Validation tick whose MuJoCo time advances, nominally 50 Hz in this scene. They are not restricted to the lower-rate global camera/robot sync point. Consequently several rows can share a sync index; elapsed simulation times strictly increase. Begin and end remain on sync boundaries. Command targets are interpreted as zero-order held from each recorded clock for replay. The export does not reconstruct intervening internal control updates, motor torques, or a continuous substep trajectory. The first sample is the explicit post-physics initial state at time zero. Version 1 trajectories contain observations only and are legacy review material, not actuator replay inputs.

The final artifact contains the settled tube's metric world pose, LEFT/BACK measurements, yaw, quaternion, dimensions, seed, preparation ordinal, initial full native qpos, outcome, and source conventions. Writes use temporary files and atomic publication without overwriting previous artifacts. File I/O runs off the simulation thread. An interrupted attempt can leave a start manifest without a completed trajectory; it is not a completed dataset episode.

Native `RecordVirtualScene` is optional and independent of trajectory export. Configure the native Observer's output and virtual cameras first, then enter the exact native session label in the task. The task must start a fresh session to track attempt ordinals. Keep native discard-failed/retry disabled and start subsequent episodes through this task. The script does not discover native output filenames or an authoritative engine episode index; the manifest's ordinal is a task-owned counter, not an engine-reported identity. Its session ID is also included in the registered native task description. Native writer finalization can independently reject an episode.

The scene's bench, Pika colour, and D405 depth views are **virtual cameras**. This script does not record or synchronize real camera video, real CAN feedback, or hardware actions. It does not implement training or an external-agent reset callback.

## Physical replay comes after review

Randomizing a virtual tube never moves the physical tube. For each later physical replay, place the real tube upright at that artifact's **actual** LEFT/BACK/yaw start and verify the first joint pose. Do not substitute the goal cross as the starting position or assume an implicit zero/home move.

The plugin's replay reader/planner validates version 2 commanded targets and applies verified hardware limits and timing constraints. It compares the physical starting state with the measured first sample separately; it does not infer a zero/home move. Hardware trajectory execution and synchronized real-sensor recording require their own backend integration; this task does not enable either. Keep simulation and physical execution separate until that path has been tested. Review failed and invalid-capture outcomes rather than feeding them to a policy as successful demonstrations.
