# Office PiPER mounting frames

The office desk is 1.40 m along world X, 0.70 m along world Z, with its top at world Y = 0.75 m. Left/right and front/back labels refer to this desk frame, not a changing camera viewpoint.

The `piper.xml` source retains the original mounting frame for Piper 1 and Piper 2. `piper_mount_180.xml` is the same PiPER + Pika model with a fixed 180-degree rotation about the MuJoCo model Z axis on `worldbody/base_link` (`quat="0 0 0 1"`). It is used only for Piper 3 and Piper 4. Joint axes, joint zero coordinates, Pika geometry, actuator parameters, and the home keyframe are unchanged. Both XML files share the same mesh resources.

This mounting rotation is baked into a distinct model because the current engine MuJoCo root supports translation but does not propagate root rotation into simulation. All four Hazel root rotations therefore remain zero. This does not change the real robots or their calibration.

| Robot | Root position in metres | Model | Base outer bounds in world X | Base outer bounds in world Z |
| --- | --- | --- | --- | --- |
| Piper 1, back left | (-0.2315, 0.75, -0.29) | piper.xml | -0.2900 to -0.1915 | -0.33 to -0.25 |
| Piper 2, front left | (-0.2315, 0.75, +0.29) | piper.xml | -0.2900 to -0.1915 | +0.25 to +0.33 |
| Piper 3, back right | (+0.6385, 0.75, -0.29) | piper_mount_180.xml | +0.5985 to +0.6970 | -0.33 to -0.25 |
| Piper 4, front right | (+0.6385, 0.75, +0.29) | piper_mount_180.xml | +0.5985 to +0.6970 | +0.25 to +0.33 |

The left pair is 41 cm from the left short edge. Matching the front-left clearance to the existing back-left clearance is an assumption until measured. All base near edges are 2 cm from their respective long edges. The right pair is 3 mm from the right short edge. Left and right columns face inward; their fixed mounting headings should be checked against the real bases during calibration.

The black desk reference cross is at world (0.205, 0.753, 0), 90.5 cm from the left short edge, 49.5 cm from the right short edge, and 35 cm from either long edge. This is a measured reference, not the geometric tabletop centre. The teal measurement overlay is visual only and has no colliders.

The home keyframe is qpos [0, 0, 0, 0, 0, 0, -0.025, +0.025] and ctrl [0, 0, 0, 0, 0, 0, -0.025]. Scene cloning/reopening compiled independent native MuJoCo instances, with all six arm joints at zero. All motion graphs are currently disabled through exact-ID scene-only gRPC calls. The active flag is runtime state and may need to be disabled again on a future scene reopen.

These separate MuJoCo instances do not provide automatic inter-robot collision prevention. The four models and measured layout are scene references. Mounted camera poses remain estimates; camera and robot calibration are separate work.
