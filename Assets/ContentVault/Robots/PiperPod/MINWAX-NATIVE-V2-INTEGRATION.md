# Clean native Minwax visual revision

`piper_tube_task_native_v2.xml` is a **new task-model revision**. It replaces the scanned tube visual with the clean native POD tube and separate cap meshes. The old qualified `piper_tube_task.xml`, its resources and recorded episodes remain unchanged. The task source verifier accepts exactly the old and new qualified XML identities, so the original scene remains usable.

## Identity and editable assets

- New XML SHA-256: `da9c70def2b1b342b96f5455f8d384e3fe39c54450b227555c07d2795457a3cc`.
- Original qualified XML SHA-256: `63a5ba5b6be49eeeb6d216594714255d61a8d50b41d4ef799fb3b8f20e519163`.
- Assets: `assets/minwax-native-v2/` contains 17 native OBJ material chunks, normals and consumer-correct UVs, two PNG maps, assembled and separate-part GLBs, previews, vector artwork/generator, and the editable native POD source package.
- **Authoritative editable Engine source:** project `Assets/Minwax-native-v2e/source/minwax-closed-assembly.hpodgraph`, registered graph handle `1959052287715013197`. Its closed/open previews were checked in the actual Engine and render the label upright.
- Portable v2e package generation: `d8bb0cf5cb9e6778803792434a0a5e8966003b18bebaa298f73e88f6a0f108e8`.
- Portable graph entry: `assets/minwax-native-v2/editable-package/generations/d8bb0cf5cb9e6778803792434a0a5e8966003b18bebaa298f73e88f6a0f108e8/sources/minwax-native-v2e-source/minwax-closed-assembly.hpodgraph`.
- Closed graph SHA-256: `59de46c939f88a64dba43e4d0cc422b95410512e2f15a13b6ed0ca19dca63f5d`.
- `native-v2-resource-manifest.json` identifies every copied resource by size and SHA-256. Its complete resource fingerprint is also embedded in the XML custom metadata, so the XML identity distinguishes this normals-corrected export from provisional geometry.

The offline artwork generator below Assets is intentionally stored as `label-artwork-source/BuildLabel.cs.txt`. The Engine auto-registers every `.cs` below Assets for runtime compilation. Its accompanying PowerShell authoring wrapper reads the text explicitly; the installed task does not compile the generator or need System.Drawing. Do not rename that archive back to `.cs` inside Assets.

This is native parametric geometry: profile/field deformation, revolve, boolean cuts, bevels, corrected normals and native materials. The scan is not used for the final visual mesh or label. It remains the source of the four already-qualified **collision hulls**, which deliberately stay unchanged.

Use **v2e** for native graph editing. The former v2c graph is an archived portable PBR reconstruction: its extended IOR/coat material vector is unsupported by this Engine build, and its image UV origin differs. v2d corrected material compatibility but retained the inverted image origin. Neither is the current Engine editing entry.

UVs intentionally differ at the consumer boundary. Native Engine image coordinates have the top at V=0, so the editable source uses `V=(193−height_above_cap_bottom_mm)/160`. The exported MJCF/OBJ uses bottom-origin V, explicitly flipped by the native exporter. All 17 chunks retain exactly the same positions, normals and topology as the earlier qualified visual export; the textured OBJ V change is at most 6e−8 after the conversion. Do not flip the PNG or change the native source back to the old V formula.

## Physics validation

`MINWAX-NATIVE-V2-PHYSICS.json` records the offline audit. Both models compile with MuJoCo 3.17: nq 15, nv 14, nu 7 and 43 bodies. All 204 audited numeric arrays for bodies/inertias, joints, actuators, constraints, sensors, sites, trees and keyframes match exactly. All 44 contact geometries and the selected compiled contact-mesh vertex/face/normal/UV/pose data match. Solver options, contact friction and the NoSlip 3 setting match.

A deterministic 500-step/1-second comparison with the same reference tabletop and controls produced exactly equal qpos, qvel, qacc and constraint forces. All 2000 contact samples had equal named pairs, distances, contact frames, friction, solver parameters and force/torque. This is a physical-equivalence regression, not a new claim of native expert-episode completion.

One explicit visual-derived exception is recorded: MuJoCo computes the tube's angular `dof_length` from all geometry bounds, including visual meshes. The new mesh chunks change that sleep velocity scale from.12757216m to.14017761m for only the three unactuated tube-rotation DOFs. Sleeping is disabled in both models, and no actuated joint changes. Visual/BVH index bookkeeping also changes with the added visual chunks. Convex-graph/polygon/BVH implementation arrays are not claimed to have been compared.

The cap is a separate editable visual part, rigidly attached to the existing tube body in this task XML. No new physical cap body, joint, mass, or collision was introduced. Unscrewing requires a later, separately qualified interaction model.

## Rendering limitations

The authoritative v2e POD material uses the Engine-supported six-float material plus albedo and packed metallic/roughness maps. It relies on the Engine's default dielectric IOR and no additional coat. The current Engine MJCF material importer reads the albedo map, forces metallic 0, and derives roughness from `1−specular`; it does not import packed MR maps. This XML therefore supplies the correct dielectric base color and matching constant roughness via that existing fallback. The label uses roughness 87/255≈.341; other materials follow their authored scalar values. Native material maps remain available in the editable package. No Engine material-import code was changed.

## Activation handoff

1. Save/checkpoint the current scene before switching models.
2. Point the existing task MuJoCo component to this new XML. Keep the calibrated root transform and existing task/camera/entity references; all robot/tube joint and body names used by the task are retained.
3. Set the new scene's task source-file field to this XML. `TubeActuatorQualification.IsQualifiedSource` now permits only its exact new hash or the exact original hash; both actual source files passed the existing private C# verifier's seven position-actuator tag/order/transmission checks. Modified files remain rejected. Do not remove the source check.
4. Check the native viewport, three virtual camera views, label orientation, closed cap and task preparation before collecting a new dataset. A new native expert-run check may then be recorded with the new model provenance.
5. Import the editable POD package as a reusable asset if desired. Do not spawn another visible tube on top of the task world's tube merely to retain the source graph.

No scene/UI control, backend restart, hardware command, or active model switch was performed during staging.
