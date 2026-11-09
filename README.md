# Office Pipers

Lucky Engine project for a measured office desk with four PiPER robots, Pika grippers, procedural office/Minwax assets, and a C# tube pick-and-place demonstration. This repository is the scene and task project; the engine and robot-control plugin are separate dependencies.

![Office Pipers scene](Thumbnail.png)

## Open the project

1. Install Git LFS, then clone with the large assets:

   ```powershell
   git lfs install
   git clone https://github.com/luckyrobots/office-pipers.git
   cd office-pipers
   git lfs pull
   powershell -NoProfile -ExecutionPolicy Bypass -File tools/Configure-Project.ps1
   ```

2. Use the current Lucky Engine development build from Lucky Robots. The tested checkout is branch `feature/plugin-sdk-pipers`, base commit `b512a9a7ee0db821c61893805cebb576f9fa4904`, **with local plugin/runtime changes**. That commit by itself is not a complete reproducibility claim. Obtain the corresponding development build/source access from Lucky Robots; engine binaries and private engine patches are not included here. The scripts require the current C# MuJoCo, motion-graph, Observer and `Hazel.Data.EpisodeFollowers` APIs.
3. Install the .NET 9 SDK if the development build does not already provide its script-build prerequisites. Open `office pipers.hproj` in Lucky Engine, regenerate the C# project through the editor, and build scripts. Generated engine DLLs, `.csproj`/`.sln` files and build output are deliberately excluded.
4. Choose a scene in `Assets/Scenes` and press **Play** for a simulation preview. The project opens the two-arm relay by default.

| Scene | Demonstration |
|---|---|
| `TubeRelayLoop.hscene` | Piper 1 moves the tube from P3 to the center cross. The opposite arm returns it to P3. |
| `TubeRelayLoopWithOneArm.hscene` | Piper 1 performs both legs, returning to joint zero after each placement. |
| `TubePickPlaceNative.hscene` | Piper 1 places the tube on the center cross and returns to joint zero. |

See [Relay scenes](RELAY.md) for loop settings and exports. `Main.hscene` is the office layout, and `TubePickPlace.hscene` is an older task scene.

The configure script only sets clone-local source-model and export paths in the task scenes. It neither starts the editor/server nor sends robot commands. Run it before opening the project, and rerun after a pull if scene paths were replaced. Export files go to the current user's local application-data directory.

## Current task setup

| Setting | Saved value |
|---|---|
| Desk section | 1400 × 700 mm |
| Fixed tube start, P3 | LEFT 900 / BACK 120 mm |
| Place target, existing desk cross | LEFT 903 / BACK 350 mm |
| Tube | 193 mm high, 43 mm round reference diameter; 0.18 kg **estimated** mass |
| Radial approach pitch | 10° |
| Forward grasp insertion | 35 mm, horizontally, with unchanged approach height |
| Open / close position targets | 80 / 30 mm; object contact keeps the measured opening larger than the close target |
| Episodes per Play run in `TubePickPlaceNative` | 1 |
| Capture | 50 Hz trajectory JSON; dataset-camera rendering off |

LEFT and BACK refer to the fixed desk edges used by the scene. They do not change with seating/viewpoint. Source tube coordinates and the goal are unchanged by the insertion adjustment. The insertion reference is carried through pickup, carry, placement and withdrawal.

On November 5, 2026, the saved 35 mm insertion / 30 mm close configuration completed a **simulation-only** pickup, placement and return to zero in 12.02 simulated seconds, exporting 606 samples with `outcome: success`. Simulated placement error was about 5.10 mm. The new physical grasp has **not yet been verified**. A previous real trajectory followed the arm path and returned to zero, but left the tube at P3; that is not a successful physical pickup.

## Files to work on

- `Assets/Scripts/Client/Source/WoodFillerDemonstrator.cs`: expert motion sequence, grasp reference, home motion and episode sequencing.
- `Assets/Scripts/Client/Source/WoodFillerTask.cs`: initial placement, task checks, episode ownership and recording/export lifecycle.
- `Assets/Scripts/Client/Source/WoodFillerLayout.cs`: deterministic start sampling and desk/base exclusions.
- `Assets/Scripts/Client/Source/WoodFillerTrajectory.cs`: measured states and actual position-command export.
- `Assets/ContentVault/Robots/PiperPod/`: robot/task MuJoCo XML, motion graph, meshes, editable tube resources and provenance.
- `Assets/Minwax-native-v2e/` and the editable POD package under `PiperPod/assets/minwax-native-v2/`: reconstructed tube/body/cap and procedural material sources. Body and cap have separate editable geometry; cap-opening interaction is future work.
- `Assets/Scenes/`: saved office/task layouts, camera entities and metric desk placement.

Keep the asset registry, IDs and relative resource paths together when moving assets. The task verifies the authored position-actuator model's pinned SHA-256. Editing the qualified XML requires deliberate requalification and an update to that source identity; do not bypass its checks merely to run an edited model.

## Plugin, hardware and cameras

The native **Pipers** panel/backend comes from [luckyrobots/piper-direct-c](https://github.com/luckyrobots/piper-direct-c); the plugin catalog is [luckyrobots/luckyengine-plugins](https://github.com/luckyrobots/luckyengine-plugins). The current local development package includes changes beyond the public backend repository's current release state. Use the package supplied for the matching engine build, or build the coordinated source revisions; no working remote plugin release is asserted here.

Simulation does not require the physical robot or its cameras. Physical control is a separate, explicit **Follow simulation** session: it matches the first pose, reports Ready, then follows when Play begins. Hardware assignments, serial/CAN identities, personal camera calibration, session ownership and credentials are not shipped in this project. Configure those on the target workstation. The four scene robot models do not mean four physical robots are connected.

The task script does not send hardware commands directly. Do not interpret simulated success, visual camera alignment, or source episode completion as proof that a physical object was grasped. The follower retimes commands under the real controller's reported limits; simulation and hardware need not finish at the same wall-clock instant.

## Data and development notes

Trajectory export defaults to `%LOCALAPPDATA%/LuckyRobots/OfficePipers/TubeEpisodes`; camera datasets use the engine's local `DataSessions` output. Both are ignored. Preview JSON contains position targets and measured simulation states, not camera images or a ready-made vision-training dataset. Enable `RecordCamerasOnPlay` in the demonstration inspector to collect the configured virtual RGB/depth streams; sign in to LuckyHub when the native recorder requires it, and choose an encoder supported by the development machine.

`ContinuousEpisodes` and `MaximumEpisodes` control repetition. For real episodes, enable `WaitForPhysicalTubePlacement` so the operator places the actual tube at each saved start. The next source episode waits for its registered physical follower to finish. Start with the fixed P3 example before expanding the random region.

See [third-party notices](THIRD_PARTY_NOTICES.md) for asset provenance and licensing boundaries. Public visibility does not assign a new blanket license to this project or its third-party assets.
