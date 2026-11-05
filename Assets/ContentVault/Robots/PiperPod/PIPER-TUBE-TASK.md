# Simulation-only wood-filler pick-and-place model

`piper_tube_task.xml` is a separate task model. It preserves the calibrated
`piper.xml` joint frames and Pika geometry without modifying that source file.
The model places the moving Piper, wood-filler tube, and three fixed zero-pose
Pipers in **one MuJoCo world**, so their contacts interact physically.

## Editor binding

Use the target robot's calibrated engine root transform:

- Position: `(-0.33811426162719727, 0.75, -0.29250943660736084)` metres.
- Rotation quaternion XYZW: `(0, -0.018902299371732233, 0, 0.9998213355787431)`.
- Uniform scale: one.

The three other robots are fixed visual and collision copies inside this model.
Remove the old three robot instances in the **task scene only** to avoid duplicate
rendering and separate physics instances. The original office scene stays intact.

Lucky Engine injects the procedural desk's MuJoCo compound box into this world.
Keep that desk collider: its half extents are `(0.7, 0.015, 0.35)` in engine
coordinates, centered at desk Y `0.735`, giving a tabletop at Y `0.75`. The XML's
`task_tabletop_collision` is a collision-disabled reference to prevent duplicate
table contacts. A standalone external MuJoCo test must enable that reference or
provide an equivalent table. The validator explicitly does this and records it.

## Object and scene coordinates

- Free body and free joint name: `wood_filler_tube`.
- Tube: Minwax Stainable Wood Filler, 6 fl oz / 177mL; 193 mm total height, black cap down.
- The reported 43mm diameter describes the round cap/body. The scanned flattened
  top is 76.56mm wide; the grasp section is 55.43 ×40.46mm at 87.815mm above the cap bottom.
- Root is geometric midheight, not a measured mass centre.
- Tube upright root Y = `0.8465` on the 0.75 m tabletop.
- Desk dimensions: 1400 × 700 mm. Engine desk centre is X = Z = 0.
- Goal cross: LEFT 903 / BACK 350 mm, engine `(0.203, 0.75, 0)`.
- Start tube root in source MJCF Z-up coordinates:
  `(0.5517838108798164, -0.27184745849768316, 0.0965)`.
- `task_grasp_site` is the pad-centre reference in `pika_base` at source local
  `(0.1795, 0, 0.0006)`. The old `grasp_site` remains unchanged for motion graphs.
  New minus old reference is `(0.0445, 0, -0.0058)` in the Pika frame.

Always address the tube body through the **owning** `MujocoSceneComponent` and its
name. A body index is local to one model; broadcasting it to other instances can
modify an unrelated robot.

## Visual provenance

The user supplied https://scaniverse.com/scan/356vimoazqajzk66 and photographs
identifying the product. The active model uses that scan's actual surface and
label texture under `assets/minwax-scan/`, with its support/table cropped away.
Height is normalized to 193mm with uniform scale0.999921. The original scan and
earlier procedural prototype remain in the authoring workspace.

The installed OBJ is a format conversion of the cropped scan into native Z-up
coordinates. The PNG is a lossless conversion of the original decoded JPEG pixels
for MuJoCo image loading; no label artwork is synthesized or retouched. The scan
contains capture lighting and reconstruction noise, so it is not a calibrated
albedo map or a claim of pixel-exact physical camera matching.

`MINWAX-SCAN-PROVENANCE.json` records the crop, dimensions and resource hashes.
The plugin/runtime needs only bundled mesh/texture files, not the offline Python
decoding or preparation tools.

## Simulation estimates and replay limits

Tube mass defaults to **0.18 kg** and sliding friction to **0.7**; these are
unmeasured estimates. Four convex scan hulls represent the cap, shoulder, body and
sealed upper sleeve. They are manually segmented overlapping convex pieces,
not direct concave visual-mesh collisions. They cannot simulate squeezing or
deformable contents.

The previous Pika finger contact boxes stopped short of the CAD jaw tips. This
task uses three contact boxes per finger following the CAD finger envelope.
Gripper stiffness is an estimated **2000 N/m**, damping **20 N·s/m**, with a
±10 N actuator limit. These are simulation settings, not measured physical Pika
force. Existing engine runtime overrides replace those authored gripper settings
in Play: gain2500, damping200, force±50N, finger armature1, damping5,
friction1.5/0.05/0.001, joint-equality solref0.002/1 and impratio1.
The offline contact qualification applies these exact overrides. None of these
are measured physical Pika force/inertia parameters.

The symmetry equality is made stiffer so the two finger positions remain
paired under load. The tube is carried by frictional finger contacts; there is
no grasp weld or scripted attachment.

The task model enables `noslip_iterations="3"`. This is MuJoCo's friction-only
post-processing solver, used to suppress soft-contact creep during slow lifts;
it does not attach the tube or increase friction/force limits. The original fast
qualification missed creep under the much slower native motion graph. Replaying
the actual failed episode's held controls reproduced the slip, while NoSlip3
retained the same 36mm closing target through the lift and a five-second hold.
See [MuJoCo's slip documentation](https://mujoco.readthedocs.io/en/3.17.0/modeling.html#preventing-slip).
NoSlip adds computation and changes the inverse-dynamics contract; it is a task
simulation choice, not measured physical gripper calibration.

Task joint ranges intersect the robot's reported limits: J1 ±150°, J2 0–180°,
J3 −154.527–0° (the source's tighter lower limit), J4 ±100°, J5 ±70°, J6 ±170°.
Pika opening is limited to 0–90 mm. Its source joint conversion is:

`opening_mm = (joint7 + 0.0470221912391186) * 2000`

These limits do not certify a recorded trajectory for physical replay. The
dataset task and its demonstrator must remain simulation-only until an explicit
separate physical replay step is enabled and verified.

## Validated first spawn region

The actual scan contact model is qualified in **LEFT 950–980 / BACK 160–220 mm**.
The bundled `MINWAX-CONTACT-QUALIFICATION.json` contains the exact sampled starts,
tube yaws, outcomes and qualified XML hash. Tests perform physical finger-contact
pickup, 142mm lift, five-second airborne hold, carry to the measured goal cross,
placement, release and settling, using the existing native runtime overrides
and the authored NoSlip3 option. Commands ramp open over0.6s, close over0.9s and
release over0.5s; approach/lift/lower/withdraw take8.64s and carry takes6s, covering
the demonstrator's1.2× movement slowdown. All72 sampled cases pass with maximum
goal error3.389mm and no MuJoCo warnings. This remains inside the25mm placement
criterion; it is not a3mm physical accuracy claim.

The20ms sampled audit covers883,991 tube contacts: maximum gripper actuator force
24.284N (within the unchanged50N simulation cap), maximum elliptic cone ratio
1.00000036, and maximum normal-equivalent cone excess3.79e-9N. Those tiny solver
residuals are reported explicitly; mass0.18kg and friction remain unmeasured
simulation assumptions. The report does not certify physical force or replay.

The validated approach is a **horizontal radial side grasp**, Pika Z upright,
with pad-centre height 87.815 mm above the desk and 80mm open /36mm close commands.
Its heading changes with the target's direction from the base; tests cover tube
yaws0–315° in45° steps without an additional tube-yaw correction. A fixed perfectly top-down pose is not
reachable everywhere within the J5 limits. This small test region is a starting
dataset region, not evidence for the whole tabletop or physical transfer.

Validation/provenance files in the authoring workspace:

- `tube-audit/piper-tube-task-manifest.json`
- `tube-audit/piper-tube-contact-validation.json`
- `tube-audit/piper-tube-place-validation.json`
- `tube-audit/piper-tube-spawn-ik-validation.json`
- `tube-audit/piper-tube-spawn-contact-validation.json`
- `tube-audit/piper-tube-grasp-orientation.json`

The raw model quaternion in the old orientation diagnostic remains useful, but
its initial full-basis engine quaternion conversion is superseded by the native
premultiply-only pose API. Motion graph orientation targets refer to `grasp_site`
in `pika_base`, whose frame is fixed −90°Y relative to `link6`. The native driver
must convert wrist orientation into the actual site orientation before sending it.
