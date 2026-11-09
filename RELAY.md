# Tube relay scenes

Run `tools/Configure-Project.ps1` from a fresh clone before opening a task
scene. The setup script resolves the model paths and assigns local export
directories.

## One-arm relay

Open `Assets/Scenes/TubeRelayLoopWithOneArm.hscene` and press **Play**. Piper 1
picks up at P3, places the tube on the center cross, and returns to joint zero.
It then picks up the same tube from the cross, returns it to P3, and homes
again. The next cycle uses the tube's settled position at P3. The other arms
are fixed scene geometry.

The **Piper 1 round trip** script exposes `ContinuousLoops` and
`MaximumEpisodes`. Exports use the single-arm version 2 format and default to
`%LOCALAPPDATA%/LuckyRobots/OfficePipers/TubeRelayLoopWithOneArm`.
The one-arm path was exercised offline with the native motion graph and
MuJoCo. Physical pickup and return are unverified.

## Two-arm relay

Open `Assets/Scenes/TubeRelayLoop.hscene`, then press **Play**. This scene is
simulation-only. It does not command the connected real arm.

One episode contains the whole round trip:

1. Both arms start at zero, with open jaws. The tube stands at P3: **BACK
   120 mm / LEFT 900 mm**.
2. Piper 1 picks up the tube and places it at the existing cross: **BACK
   350 mm / LEFT 903 mm**. Piper 1 returns home before the next arm moves.
3. Piper 3, on the opposite back edge, picks up that same tube, returns it to
   P3, and returns home.
4. The episode finishes and exports both robot tracks. The next round trip
   starts only after completion and any registered follower acknowledgements.

There is no tube reset or recording boundary between the two pickups. Between
episodes, the normal task preparation restores the measured P3 start. A missed
grasp finishes the current arm's path to zero and marks the episode unsuccessful;
it does not hand an absent tube to the other arm or restart indefinitely.

Select **Round trip - P3 to cross to P3 - simulation only** in the hierarchy:

- `ContinuousLoops` enables repetition.
- `MaximumEpisodes=0` means repeat until stopped. Set a positive number for a
  finite test.
- `RecordCamerasOnPlay` adds the engine's configured virtual camera streams.
  Keep it off for a fast trajectory-only preview.

The two `WoodFillerDemonstrator` components hold per-arm choreography settings:
`TravelSpeed` and `VerticalSpeed` are metres per simulation second;
`AngularSpeed` is radians per simulation second. These do not override real
hardware limits. The original `TubePickPlaceNative` scene keeps the working
single-arm physical demo.

The return arm is articulated in the same MuJoCo world under `piper3_` names.
Its mounting pose is authored from the existing scene and still requires a
separate physical calibration before a second real arm is connected.

After lifting, the relay measures the tube's actual offset in the jaws and
uses that reference for carry and placement. It changes the tool targets;
it does not teleport the tube. Read-only release/withdraw diagnostics on each
arm help distinguish grasp offset from a later contact push.

Exports use `kind: lucky.piper.multi-robot-trajectory`, **version 3**. Both
`robotTracks` share the episode, simulation timestamps and sync indices, while
each retains its six measured joints, six actuator commands, measured jaw gap
and jaw command. The distinct kind prevents treating only the first arm as a
complete legacy hardware replay. Existing single-arm exports remain version 2.

The two-arm scene uses `piper_tube_relay.xml` and exports under
`%LOCALAPPDATA%/LuckyRobots/OfficePipers/TubeRelayLoop` by default. Source
SHA-256, fourteen position actuator names and order, transmissions, and
ranges are checked before recording. Model changes require requalification.

Source entry points:

- `TubeRoundTripDemonstrator.cs`: episode and arm handoff orchestration.
- `TubeRelaySequence.cs`: completion, home and recording barriers.
- `WoodFillerDemonstrator.cs`: movement, grasp, release and homing.
- `WoodFillerTask.cs`: one recording across both legs and placement checks.
- `WoodFillerTrajectory.cs`: synchronized multi-robot exports.
- `MG_PiperRelay.hpodgraph`: `ReturnArm` chooses the active IK chain.
