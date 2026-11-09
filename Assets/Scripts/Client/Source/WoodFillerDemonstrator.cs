using System;
using System.Collections.Generic;
using System.Diagnostics;
using Hazel;

namespace OfficePipers
{
	public class WoodFillerDemonstrator : Entity
	{
		[Group("Simulation demonstration")]
		public Entity? TaskEntity;
		[Tooltip("Optional shared MuJoCo world. Leave empty for the original single-arm driver attached to its own world.")]
		public Entity? PhysicsWorldEntity;
		[Tooltip("Optional motion-graph owner. A relay's two arms share one world and one graph; leave empty to use the physics-world entity.")]
		public Entity? MotionControllerEntity;
		[Tooltip("Empty for the original arm; piper3_ for the opposite arm in the round-trip world.")]
		public string JointPrefix = "";
		public string MotionInputPrefix = "";
		public int ActuatorOffset;
		[Tooltip("On Play, prepare a random tube start, home the simulated arm and run pick/place/home. Never commands the real robot directly. Turn off for manual demonstrations.")]
		public bool AutoRunOnPlay = true;
		[Tooltip("After a successful placement, return all six joints to zero, finish the recording/export, then prepare a new random start. A failure or manual Stop ends the loop.")]
		public bool ContinuousEpisodes = true;
		[Tooltip("0 repeats until stopped. A positive value stops after this many successful episodes in the current Play run.")]
		public int MaximumEpisodes;
		[Tooltip("Enable when physically following the simulation: wait at every prepared start until you place the real tube at the displayed mark and confirm. Randomization moves only the simulated tube.")]
		public bool WaitForPhysicalTubePlacement;
		[ReadOnly] public int CompletedEpisodes;
		[Tooltip("Off: real-time preview with 50 Hz joint trajectory export. On: also record every configured virtual RGB/depth frame; rendering can run slower than real time. Choose before Play.")]
		public bool RecordCamerasOnPlay = false;
		[ReadOnly] public string CaptureStatus = "Preview: trajectory export on, dataset cameras off.";
		[ReadOnly] public string Status = "Press Play to run simulated pick/place/home episodes.";
		[ReadOnly] public string LastGraspFailure = "";
		[Group("Placement diagnostics · world metres")]
		[ReadOnly] public Vector3 TubeBeforeRelease;
		[ReadOnly] public Vector3 TubeAfterRelease;
		[ReadOnly] public Vector3 TubeAfterReleaseSettled;
		[ReadOnly] public Vector3 TubeAfterWithdraw;
		[ReadOnly] public Vector3 TubeAtPlacementCheck;
		[ReadOnly] public float BeforeReleaseErrorMm = -1f, AfterReleaseErrorMm = -1f, AfterReleaseSettledErrorMm = -1f;
		[ReadOnly] public float AfterWithdrawErrorMm = -1f, PlacementCheckErrorMm = -1f;
		[ReadOnly] public Vector3 MeasuredGraspReferenceInWrist;
		[ReadOnly] public float MeasuredGraspCorrectionMm;
		[Group("Simulation demonstration")]
		[Tooltip("Set only after checking this approach in the merged simulation model. The controller refuses an unverified default approach.")]
		public bool ApproachOrientationVerified = true;
		[Tooltip("Keep the jaw-closing direction horizontal and approach radially from the base. RadialApproachPitchDegrees tilts the fingers downward when a horizontal grasp would reach the wrist limit.")]
		public bool RadialSideGrasp = true;
		[Tooltip("Downward pitch of a radial side grasp, in degrees. 0 preserves the horizontal approach. Check the complete simulated pickup/place/home before physically following a new value; it does not change the grasp point or joint limits.")]
		public float RadialApproachPitchDegrees;
		[Tooltip("Grasp insertion in millimetres along the fingers' desk-horizontal direction. 0 preserves the original grasp. The same tool reference follows pickup, carry, placement and withdrawal; object marks and calibration stay unchanged. Recheck the full simulation after changing this.")]
		public float GraspForwardOffsetMm;
		public Vector3 GraspOrientationDegrees = new Vector3(90f, 0f, 0f);
		[Tooltip("Cartesian travel speed in metres per simulation second. This changes the simulated demonstration, not the hardware speed setting.")]
		public float TravelSpeed = 0.25f;
		public float VerticalSpeed = 0.16f;
		[Tooltip("Orientation speed in radians per simulation second.")]
		public float AngularSpeed = 1.6f;
		[Tooltip("Multiplier for planned movement durations. 1 runs the configured motion speeds; larger values slow the demonstration.")]
		public float DurationScale = 1.0f;
		public float HoverHeight = 0.120f;
		public float GraspOffsetY = -0.008685f;
		public float OpenMillimetres = 80f;
		public float CloseMillimetres = 36f;
		public float PositionTolerance = 0.020f;
		public float GraspPositionTolerance = 0.005f;
		public float OrientationToleranceRadians = 0.20f;
		public float ExtraArrivalSeconds = 3f;
		public string MotionGraphSite = "grasp_site";
		public string ContactSite = "task_grasp_site";
		public string WristBody = "link6";
		public string GripperActuator = "gripper";
		[Tooltip("Measured joint tolerance for zero. Position-servo gravity/load error means the measured angles need not be mathematically exact zero.")]
		public float HomeToleranceDegrees = 1f;
		[Group("Playback diagnostics")]
		[ReadOnly] public float ElapsedSimulationSeconds;
		[ReadOnly] public float ElapsedWallSeconds;
		[Tooltip("Simulation seconds per wall-clock second during this demonstration. 1 is real time; below 1 means the engine is falling behind.")]
		[ReadOnly] public float RealtimeFactor;

		private enum StepKind { Move, Gripper, Wait, VerifyLift, VerifyCarry, VerifyPlacement, Home, Finish }
		private struct Step
		{
			public StepKind Kind;
			public Vector3 Position, Rotation;
			public float Duration, OpeningMm, PositionTolerance;
			public float MotionSpeed;
			public Quaternion WristRotation;
			public bool AppliesGraspOffset;
			public string Label;
		}
		private static readonly Identifier k_Position = new Identifier("Target_Position");
		private static readonly Identifier k_Rotation = new Identifier("Target_Orientation");
		private static readonly Identifier k_Duration = new Identifier("Duration");
		private static readonly Identifier k_Start = new Identifier("Start");
		private readonly Queue<Step> m_Queue = new();
		private WoodFillerTask? m_Task;
		private MujocoSceneComponent? m_World;
		private RobotControllerComponent? m_Robot;
		private MujocoActuator? m_Gripper;
		private uint m_MotionSite, m_ContactSite, m_Wrist;
		private float m_MaximumOpening;
		private float m_Elapsed;
		private bool m_Started, m_Busy;
		private int m_PhaseOrdinal;
		private Vector3 m_PlannedPosition, m_PlannedRotation, m_GraphSiteOffsetInWrist, m_GraspForwardOffsetInWrist;
		private Quaternion m_GraphSiteRotationInWrist;
		private float m_InitialTubeY;
		private float m_PositionTolerance, m_OrientationTolerance, m_ExtraArrivalSeconds;
		private float m_DurationScale, m_GripperStartControl, m_GripperTargetControl;
		private double m_StartSimulationTime;
		private long m_StartWallTimestamp;
		private readonly TubePlayStartup m_PlayStartup = new();
		private readonly TubeTrialOutcome m_TrialOutcome = new();
		private WoodFillerTask? m_AutomaticBeginTask;
		private int m_AutomaticBeginRevision;
		private ulong m_AutomaticBeginWorld;
		private readonly MujocoActuator?[] m_Arm = new MujocoActuator?[6];
		private readonly uint[] m_HomePositionAddresses = new uint[6], m_HomeVelocityAddresses = new uint[6];
		private readonly float[] m_HomeStart = new float[6];
        private bool m_Homing, m_HomeReady, m_ArmScriptOwned, m_GripperScriptOwned, m_PlacementConfirmed;
		private float m_HomeElapsed, m_HomeDuration, m_HomeStable, m_HomeStartGripper, m_HomeTargetGripper;
		private uint m_HomeFirstFingerAddress, m_HomeSecondFingerAddress;
		private int m_HomeControlRevision;
		public bool IsBusy => m_Busy;
		public bool HomeReady => m_HomeReady && !m_Homing;
		public bool RelayLegComplete { get; private set; }
		public bool RelayLegSuccessful { get; private set; }
		public bool RelayLegFaulted { get; private set; }
		private bool m_RelayManaged;
		private bool m_RecordedBeforeRelease;
		private Vector3 m_RelayPickup, m_RelayGoal;
		private ulong PhysicsWorldId => PhysicsWorldEntity?.ID ?? ID;
		private string Named(string name) => JointPrefix + name;
		private MujocoSceneComponent? ResolveWorld() => PhysicsWorldEntity?.GetComponent<MujocoSceneComponent>() ?? GetComponent<MujocoSceneComponent>();
		private RobotControllerComponent? ResolveController() => MotionControllerEntity?.GetComponent<RobotControllerComponent>()
			?? PhysicsWorldEntity?.GetComponent<RobotControllerComponent>() ?? GetComponent<RobotControllerComponent>();

		// Called by the round-trip coordinator after its task has opened this leg's goal.
		// A leg runs the exact same pickup/release/withdraw/home queue as the original
		// demonstration, but does not close the encompassing round-trip episode.
		public bool StartRelayLeg(Vector3 pickupCentre, Vector3 goalCentre)
		{
			if (m_Busy || m_Homing || !Finite(pickupCentre) || !Finite(goalCentre)) return false;
			m_RelayManaged = true;
			m_PlayStartup.Cancel();
			m_RelayPickup = pickupCentre;
			m_RelayGoal = goalCentre;
			RelayLegComplete = false;
			RelayLegSuccessful = false;
			RelayLegFaulted = false;
			StartDemonstration(automatic: true);
			if (!m_Busy) RelayLegComplete = true;
			return m_Busy;
		}

		public bool HomeRelayArm()
		{
			if (m_Busy || m_Homing) return false;
			m_RelayManaged = true;
			m_PlayStartup.Cancel();
			RelayLegFaulted = false;
			m_Task = TaskEntity?.As<WoodFillerTask>();
			BeginHoming();
			return m_Homing;
		}

		private static bool Finite(Vector3 point) => float.IsFinite(point.X) && float.IsFinite(point.Y) && float.IsFinite(point.Z);

		[Button("Run simulated demonstration")]
		[Tooltip("Simulation only. No hardware connection or robot server is used. Fails the episode if the graph cannot reach a target or the physical simulated grasp fails.")]
		public void RunSimulatedDemonstration()
		{
			StartDemonstration(automatic: false);
		}

		[Button("Real tube is at the displayed start — continue")]
		public void ConfirmTubeAtStart()
		{
			WoodFillerTask? task = TaskEntity?.As<WoodFillerTask>();
			if (!WaitForPhysicalTubePlacement || task?.EpisodeStage != TubeEpisodeStage.Ready || !m_HomeReady || m_Homing)
			{
				Status = "Wait until the start is prepared and the simulated arm is at zero, then confirm the real tube's position.";
				return;
			}
			m_PlacementConfirmed = true;
		}

		private void StartDemonstration(bool automatic)
		{
			// A manual Run consumes the automatic attempt, even if its prerequisites fail.
			if (!automatic)
			{
				if (!m_PlayStartup.Finished) CancelAutomaticEpisode("Automatic start cancelled by manual Run.");
				m_PlayStartup.Cancel();
			}
			if (m_Busy || m_Homing)
			{
				Status = "A simulated demonstration is already running.";
				return;
			}
			m_Task = TaskEntity?.As<WoodFillerTask>();
			m_World = ResolveWorld();
			m_Robot = ResolveController();
			if (m_Task == null || !m_Task.EpisodeActive || m_Task.PreparedPhysicsWorldEntityId != PhysicsWorldId || m_World == null || m_Robot == null)
			{
				Status = "Assign TaskEntity, attach this driver to its shared Piper/tube physics world, and start a task episode first.";
				return;
			}
			if (!ApproachOrientationVerified || !ValidSettings())
			{
				Status = "The approach orientation/limits must be verified in this model before the demonstration can run.";
				return;
			}
			m_MotionSite = m_World.GetSiteID(Named(MotionGraphSite));
			m_ContactSite = m_World.GetSiteID(Named(ContactSite));
			m_Wrist = m_World.GetBodyID(Named(WristBody));
			m_Gripper = m_World.GetActuator(Named(GripperActuator));
			uint finger = m_World.GetJointID(Named("joint7"));
			if (m_MotionSite == uint.MaxValue || m_ContactSite == uint.MaxValue || m_Wrist == uint.MaxValue || m_Gripper == null || finger == uint.MaxValue
				|| !m_World.GetJointRange(finger, out float minimum, out float maximum) || !float.IsFinite(minimum) || !float.IsFinite(maximum) || minimum >= maximum || maximum > 0.000001f)
			{
				Status = "The shared model is missing the graph/contact sites, wrist, or canonical Pika actuator mapping.";
				return;
			}
			m_MaximumOpening = -2000f * minimum;
			m_PositionTolerance = PositionTolerance;
			m_OrientationTolerance = OrientationToleranceRadians;
			m_ExtraArrivalSeconds = ExtraArrivalSeconds;
			m_DurationScale = DurationScale;
			float allowedMaximumOpening = MathF.Min(90f, m_MaximumOpening + 2000f * maximum);
			float allowedMinimumOpening = MathF.Max(0f, m_MaximumOpening + 2000f * minimum);
			if (OpenMillimetres > allowedMaximumOpening || CloseMillimetres > allowedMaximumOpening
				|| OpenMillimetres < allowedMinimumOpening || CloseMillimetres < allowedMinimumOpening)
			{
				Status = "Configured jaw opening is beyond the verified model travel.";
				return;
			}
			Quaternion wristRotation = m_World.GetOrientation(m_Wrist);
			if (!TrySiteWorldPose(m_MotionSite, out Vector3 motionSite, out Quaternion siteRotation)
				|| !TrySiteWorldPose(m_ContactSite, out Vector3 contactSite, out _))
			{
				Status = "Cannot read the motion/contact site pose from the shared model.";
				return;
			}
			m_GraphSiteOffsetInWrist = wristRotation.Conjugate * (motionSite - contactSite);
			m_GraphSiteRotationInWrist = wristRotation.Conjugate * siteRotation;
			m_PlannedPosition = motionSite;
			m_PlannedRotation = siteRotation.EulerAngles;
			Vector3 pick = (m_RelayManaged ? m_RelayPickup : m_Task.PreparedTubeCentre) + new Vector3(0f, GraspOffsetY, 0f);
			Vector3 place = (m_RelayManaged ? m_RelayGoal : m_Task.PreparedGoalCentre) + new Vector3(0f, GraspOffsetY + 0.002f, 0f);
			Vector3 graspRotation = ApproachRotation(pick);
			Vector3 placeRotation = ApproachRotation(place);
			if (!TubeGraspGeometry.TryForwardOffsetInWrist(new Quaternion(graspRotation), GraspForwardOffsetMm, out m_GraspForwardOffsetInWrist))
			{
				Status = "Grasp insertion must be 0–50 mm and the fingers must have a horizontal forward direction. Nothing was queued.";
				return;
			}
			m_InitialTubeY = m_Task.PreparedTubeCentre.Y;
			m_TrialOutcome.Reset();
			LastGraspFailure = "";
			m_RecordedBeforeRelease = false;
			TubeBeforeRelease = TubeAfterRelease = TubeAfterReleaseSettled = TubeAfterWithdraw = TubeAtPlacementCheck = Vector3.Zero;
			BeforeReleaseErrorMm = AfterReleaseErrorMm = AfterReleaseSettledErrorMm = AfterWithdrawErrorMm = PlacementCheckErrorMm = -1f;
			MeasuredGraspReferenceInWrist = Vector3.Zero;
			MeasuredGraspCorrectionMm = 0f;
			float safeY = MathF.Max(contactSite.Y, MathF.Max(pick.Y, place.Y) + HoverHeight);
			m_Queue.Clear();
			AppendGripper(OpenMillimetres, 0.6f, $"Open jaws to {OpenMillimetres:F0} mm");
			AppendMove(new Vector3(contactSite.X, safeY, contactSite.Z), wristRotation.EulerAngles, VerticalSpeed, "Lift clear before turning", applyGraspOffset: false);
			bool returningToP3 = m_RelayManaged && m_Task.SingleArmRoundTrip && m_RelayGoal.Equals(m_Task.PreparedStartCentre);
			// Coming straight from zero to the centre can switch IK branches.
			if (returningToP3)
				AppendMove(new Vector3(place.X, safeY, place.Z), placeRotation, TravelSpeed, "Travel via the P3 hover point");
			AppendMove(new Vector3(pick.X, safeY, pick.Z), graspRotation, TravelSpeed, "Travel above tube");
			AppendMove(pick, graspRotation, VerticalSpeed, "Approach tube", GraspPositionTolerance);
			AppendGripper(CloseMillimetres, 0.9f, "Close on tube");
			AppendMove(new Vector3(pick.X, safeY, pick.Z), graspRotation, VerticalSpeed, "Lift tube");
			m_Queue.Enqueue(new Step { Kind = StepKind.VerifyLift, Label = "Check simulated grasp", Duration = 0.05f });
			AppendMove(new Vector3(place.X, safeY, place.Z), placeRotation, TravelSpeed, returningToP3 ? "Carry tube to P3" : "Carry tube to cross");
			m_Queue.Enqueue(new Step { Kind = StepKind.VerifyCarry, Label = "Confirm tube is still held before lowering", Duration = 0.05f });
			AppendMove(place, placeRotation, VerticalSpeed, returningToP3 ? "Lower tube onto P3" : "Lower tube onto cross", GraspPositionTolerance);
			m_Queue.Enqueue(new Step { Kind = StepKind.Wait, Duration = 0.4f, Label = "Settle before release" });
			for (int i = 1; i <= 5; i++)
			{
				AppendGripper(CloseMillimetres + (OpenMillimetres - CloseMillimetres) * (i / 5f), 0.1f, "Release gently");
			}
			m_Queue.Enqueue(new Step { Kind = StepKind.Wait, Duration = 0.7f, Label = "Wait for tube to settle" });
			AppendMove(new Vector3(place.X, safeY, place.Z), placeRotation, VerticalSpeed, "Withdraw jaws");
			m_Queue.Enqueue(new Step { Kind = StepKind.VerifyPlacement, Duration = 2f, Label = returningToP3 ? "Validate placement at P3" : "Validate placement at cross" });
			m_Queue.Enqueue(new Step { Kind = StepKind.Home, Label = "Return all six joints to zero" });
			m_Queue.Enqueue(new Step { Kind = StepKind.Finish, Duration = 2f, Label = "Finish the placement and home episode" });
			m_Started = false;
			m_PhaseOrdinal = 0;
			m_Busy = true;
			m_HomeReady = false;
			m_StartSimulationTime = m_World.MujocoTime;
			m_StartWallTimestamp = Stopwatch.GetTimestamp();
			ElapsedSimulationSeconds = 0f;
			ElapsedWallSeconds = 0f;
			RealtimeFactor = 0f;
			Status = "Simulated demonstration queued. The physical robot is not connected to this driver.";
		}

		[Button("Stop simulated demonstration")]
		public void StopDemonstration()
		{
			CancelAutomaticEpisode("Stopped by user before the automatic demonstration started.");
			m_PlayStartup.Cancel();
			Abort("Stopped by user.");
		}

		protected override void OnUpdate(float deltaTime)
		{
			if (m_Busy && m_World != null)
			{
				ElapsedSimulationSeconds = (float)(m_World.MujocoTime - m_StartSimulationTime);
				ElapsedWallSeconds = (float)Stopwatch.GetElapsedTime(m_StartWallTimestamp).TotalSeconds;
				RealtimeFactor = ElapsedWallSeconds > 0f ? ElapsedSimulationSeconds / ElapsedWallSeconds : 0f;
			}
			if (m_Homing)
			{
				if (AdvanceHoming(deltaTime) && m_Busy) CompleteStep();
				return;
			}
			AdvancePlayStartup(deltaTime);
			if (!m_Busy)
			{
				return;
			}
			if (m_Task == null || !m_Task.EpisodeActive)
			{
				Abort("Task episode ended; stopped the simulated demonstration.");
				return;
			}
			if (!m_Task.CaptureValid)
			{
				Abort("Simulated trajectory capture failed: " + m_Task.CaptureError);
				return;
			}
			if (m_Queue.Count == 0)
			{
				m_Busy = false;
				return;
			}
			Step step = m_Queue.Peek();
			if (!m_Started)
			{
				string phaseKind = step.Kind switch
				{
					StepKind.Move => "arm", StepKind.Gripper => "gripper", StepKind.Wait => "wait",
					StepKind.VerifyLift => "verify-lift", StepKind.VerifyCarry => "verify-carry", StepKind.VerifyPlacement => "verify-place",
					StepKind.Home => "home", _ => "finish"
				};
				if (m_RelayManaged && m_Task.TwoRobotRelay) phaseKind = (JointPrefix.Length == 0 ? "relay-outbound-" : "relay-return-") + phaseKind;
				if (!m_Task.PublishPhase($"{m_TrialOutcome.PhaseKind(phaseKind)}:{++m_PhaseOrdinal}"))
				{
					Abort("The linked episode stopped. No further simulated task movement was issued.");
					return;
				}
				m_Elapsed = 0f;
				m_Started = true;
				Status = TrialStatus(step.Label);
				if (step.Kind == StepKind.Move)
				{
					Send(step.Position, step.Rotation, step.Duration);
					ReleaseArmToGraph();
				}
				else if (step.Kind == StepKind.Home)
				{
					BeginHoming();
					return;
				}
				else if (step.Kind == StepKind.Gripper)
				{
					if (step.Label == "Release gently" && !m_RecordedBeforeRelease)
					{
						RecordTubeDiagnostic(0);
						m_RecordedBeforeRelease = true;
					}
					m_Gripper!.IK = false;
                    m_GripperScriptOwned = true;
					m_GripperStartControl = (float)m_Gripper.Value;
					m_GripperTargetControl = (step.OpeningMm - m_MaximumOpening) / 2000f;
					if (!float.IsFinite(m_GripperStartControl))
					{
						Abort("The current gripper position command is not finite.");
						return;
					}
				}
			}
			m_Elapsed += deltaTime;
			if (step.Kind == StepKind.Gripper)
			{
				float fraction = Math.Clamp(m_Elapsed / step.Duration, 0f, 1f);
				m_Gripper!.Value = m_GripperStartControl + (m_GripperTargetControl - m_GripperStartControl) * fraction;
			}
			if (step.Kind == StepKind.Move)
			{
				if (!TrySiteWorldPose(m_MotionSite, out Vector3 actualPosition, out Quaternion actual))
				{
					Abort("The motion graph end-effector site is no longer available.");
					return;
				}
				float error = (actualPosition - step.Position).Length();
				Quaternion requested = new Quaternion(step.Rotation);
				float dot = MathF.Abs(actual.X * requested.X + actual.Y * requested.Y + actual.Z * requested.Z + actual.W * requested.W);
				float angle = 2f * MathF.Acos(Math.Clamp(dot, 0f, 1f));
				if (m_Elapsed >= step.Duration && error <= step.PositionTolerance && angle <= m_OrientationTolerance)
				{
					if (step.Label == "Withdraw jaws") RecordTubeDiagnostic(3);
					CompleteStep();
				}
				else if (m_Elapsed > step.Duration + m_ExtraArrivalSeconds)
				{
					Abort($"Motion graph did not reach '{step.Label}': position error {error * 1000f:F0} mm / orientation error {angle * 180f / MathF.PI:F1} deg.");
				}
			}
			else if (step.Kind == StepKind.VerifyLift)
			{
				float height = m_Task.PreparedTubeCentre.Y;
				if (!float.IsFinite(height))
				{
					Abort("Cannot verify the simulated lift: tube feedback is not finite.");
					return;
				}
				if (height < m_InitialTubeY + 0.040f)
				{
					RecordExpectedMiss("The simulated tube did not lift with the jaws.");
				}
				else if (m_RelayManaged && !RetargetRelayToMeasuredGrasp())
				{
					Abort("The relay lift did not provide a finite, bounded object-to-tool reference. No further relay movement was queued.");
					return;
				}
				CompleteStep();
			}
			else if (step.Kind == StepKind.VerifyCarry)
			{
				uint first = m_World!.GetJointID(Named("joint7")), second = m_World.GetJointID(Named("joint8"));
				uint firstAddress = first == uint.MaxValue ? uint.MaxValue : m_World.GetJointQposAdr(first);
				uint secondAddress = second == uint.MaxValue ? uint.MaxValue : m_World.GetJointQposAdr(second);
				if (firstAddress == uint.MaxValue || secondAddress == uint.MaxValue ||
					!TrySiteWorldPose(m_ContactSite, out Vector3 contact, out _))
				{
					Abort("Cannot verify the simulated grasp before lowering: jaw or contact-site feedback is missing.");
					return;
				}
				Vector3 tube = m_Task.PreparedTubeCentre;
				float opening = m_MaximumOpening + 1000f * (m_World.GetJointPosition(firstAddress) - m_World.GetJointPosition(secondAddress));
				Vector3 expectedObjectPoint = TubeGraspGeometry.ObjectPointFromContact(contact, m_World.GetOrientation(m_Wrist), m_GraspForwardOffsetInWrist);
				float separation = (tube - expectedObjectPoint).Length();
				float maximumSeparation = m_Task.TubeDiameter * 0.5f + MathF.Abs(GraspOffsetY) + GraspPositionTolerance;
				if (!float.IsFinite(tube.Y) || !float.IsFinite(opening) || !float.IsFinite(separation) || !float.IsFinite(maximumSeparation))
				{
					Abort("Cannot verify the simulated carry: tube, jaw or contact-site feedback is not finite.");
					return;
				}
				if (!TubeGraspCheck.IsHeld(tube.Y, m_InitialTubeY, opening, CloseMillimetres, separation, maximumSeparation))
				{
					RecordExpectedMiss($"The simulated tube was lost during carry: jaw opening {opening:F1} mm / close target {CloseMillimetres:F1} mm; tube-to-grasp distance {separation * 1000f:F0} mm; lift {(tube.Y - m_InitialTubeY) * 1000f:F0} mm.");
				}
				CompleteStep();
			}
			else if (step.Kind == StepKind.VerifyPlacement)
			{
				RecordTubeDiagnostic(4);
				if (m_Task.SimulatedPlacementComplete) CompleteStep();
				else if (m_Elapsed >= step.Duration)
				{
					RecordExpectedMiss("The released tube did not settle upright at the cross.");
					CompleteStep();
				}
			}
			else if (step.Kind == StepKind.Finish)
			{
				var outcome = m_TrialOutcome.Resolve(m_HomeReady, m_Task.SimulatedPlacementComplete);
				if (m_RelayManaged && outcome != TubeTrialResult.NotReady)
				{
					m_Busy = false;
					m_Queue.Clear();
					RelayLegComplete = true;
					RelayLegSuccessful = outcome == TubeTrialResult.Successful;
					Status = RelayLegSuccessful ? "Relay leg complete; arm at zero. The shared episode remains open."
						: $"Missed grasp; arm returned to zero. {m_TrialOutcome.Reason}";
					return;
				}
				if (outcome == TubeTrialResult.Unsuccessful)
				{
					m_Busy = false;
					m_Queue.Clear();
					m_PlayStartup.Cancel();
					m_Task.FinishUnsuccessfully();
					Status = $"Unsuccessful trial; arm returned to zero with jaws open. {m_TrialOutcome.Reason} No next episode will start.";
				}
				else if (outcome == TubeTrialResult.Successful)
				{
					m_Busy = false;
					m_Queue.Clear();
					m_Task.FinishSuccessfully();
					m_PlayStartup.EpisodeFinished(m_Task.ControlRevision);
					Status = "Tube placed and arm at zero. Finalizing the recording and trajectory export before another start.";
				}
				else if (m_Elapsed >= step.Duration)
				{
					Abort("The released tube did not settle upright at the cross. Episode is unsuccessful.");
				}
			}
			else if (m_Elapsed >= step.Duration)
			{
				if (step.Kind == StepKind.Gripper && step.Label == "Release gently" && step.OpeningMm >= OpenMillimetres) RecordTubeDiagnostic(1);
				if (step.Kind == StepKind.Wait && step.Label == "Wait for tube to settle") RecordTubeDiagnostic(2);
				CompleteStep();
			}
		}

		private void AdvancePlayStartup(float deltaTime)
		{
			if (m_RelayManaged || m_PlayStartup.Finished)
			{
				return;
			}
			WoodFillerTask? task = TaskEntity?.As<WoodFillerTask>();
			TubePlayCommand command = m_PlayStartup.Advance(AutoRunOnPlay, task != null,
				task?.Workflow == TubeWorkflow.SimulationOnly, task?.EpisodeStage ?? TubeEpisodeStage.WaitingForPlay,
				TaskEntity?.ID ?? 0, task?.ControlRevision ?? 0, task?.Status ?? "Assign TaskEntity to the wood-filler episode controls.", deltaTime,
				homeReady: m_HomeReady, continuous: ContinuousEpisodes, episodeSuccessful: task?.EpisodeSucceeded ?? false,
				recordingCanContinue: task?.RecordingCanContinue ?? true, waitForPlacement: WaitForPhysicalTubePlacement,
				placementConfirmed: m_PlacementConfirmed, maximumEpisodes: MaximumEpisodes,
				followerStatus: task?.FollowerStatus ?? Hazel.Data.EpisodeFollowerStatus.NotRequired,
				followerCanAdvance: task?.FollowerCanAdvance ?? true);
			CompletedEpisodes = m_PlayStartup.CompletedEpisodes;
			if (!m_Busy) Status = m_PlayStartup.Status;
			if (command == TubePlayCommand.Prepare)
			{
				m_PlacementConfirmed = false;
				m_HomeReady = false;
				task!.RecordVirtualScene = RecordCamerasOnPlay;
				CaptureStatus = task.RecordVirtualScene
					? "Recording virtual cameras and trajectory; every configured frame is preserved."
					: "Preview: trajectory export on, dataset cameras off.";
				task!.PrepareNextEpisode();
				m_PlayStartup.AcceptOwnControlRevision(task.ControlRevision);
			}
			else if (command == TubePlayCommand.Home)
			{
				m_Task = task;
				BeginHoming();
			}
			else if (command == TubePlayCommand.Begin)
			{
				task!.BeginSimulationEpisode();
				m_PlayStartup.AcceptOwnControlRevision(task.ControlRevision);
				m_AutomaticBeginTask = task;
				m_AutomaticBeginRevision = task.ControlRevision;
				m_AutomaticBeginWorld = task.PreparedPhysicsWorldEntityId;
			}
			else if (command == TubePlayCommand.Run)
			{
				StartDemonstration(automatic: true);
				if (!m_Busy)
				{
					CancelAutomaticEpisode("Automatic demonstration could not run: " + Status);
					m_PlayStartup.Cancel();
				}
				else m_AutomaticBeginTask = null; // Running motion now owns normal Stop/Finish handling.
			}
			else if (m_PlayStartup.Finished)
			{
				CancelAutomaticEpisode(Status);
			}
		}

		private void CancelAutomaticEpisode(string reason)
		{
			m_AutomaticBeginTask?.CancelAutomaticBegin(m_AutomaticBeginRevision, m_AutomaticBeginWorld, reason);
			m_AutomaticBeginTask = null;
		}

		private void BeginHoming()
		{
			if (m_Task == null || !m_Task.PreparedSourceQualified || m_Task.PreparedPhysicsWorldEntityId != PhysicsWorldId
				|| (m_Task.EpisodeStage != TubeEpisodeStage.Ready && m_Task.EpisodeStage != TubeEpisodeStage.Active)
				|| !ValidSettings())
			{
				Abort("Homing requires this task's qualified simulation world and a ready or active episode.");
				return;
			}
			m_World = ResolveWorld();
			m_Robot = ResolveController();
			if (m_World == null || m_Robot == null)
			{
				Abort("The task world is missing its simulation model or motion graph.");
				return;
			}
			for (int i = 0; i < m_Arm.Length; i++)
			{
				string name = Named("joint" + (i + 1));
				uint joint = m_World.GetJointID(name), actuator = m_World.GetActuatorIndex(name);
				m_Arm[i] = m_World.GetActuator(name);
				if (joint == uint.MaxValue || actuator != (uint)(ActuatorOffset + i) || m_Arm[i] == null || m_World.GetJointType(joint) != 3
					|| m_World.GetActuatorTrnJointID(actuator) != (int)joint || m_World.IsActuatorClaimedByPolicy(actuator)
					|| !m_World.GetJointRange(joint, out float lo, out float hi) || lo > 0f || hi < 0f
					|| !m_World.GetActuatorCtrlRange(actuator, out float ctrlLo, out float ctrlHi) || ctrlLo > 0f || ctrlHi < 0f
					|| (!m_ArmScriptOwned && !m_Arm[i]!.IK))
				{
					Abort("Cannot home: the six named position servos must allow zero and must not belong to another script or policy.");
					return;
				}
				m_HomePositionAddresses[i] = m_World.GetJointQposAdr(joint);
				m_HomeVelocityAddresses[i] = m_World.GetJointQvelAdr(joint);
				m_HomeStart[i] = (float)m_Arm[i]!.Value;
				if (m_HomePositionAddresses[i] == uint.MaxValue || m_HomeVelocityAddresses[i] == uint.MaxValue
					|| !float.IsFinite(m_HomeStart[i]) || m_HomeStart[i] < ctrlLo || m_HomeStart[i] > ctrlHi)
				{
					Abort("Cannot home: a starting position command is unavailable or outside the qualified model range.");
					return;
				}
			}
			m_Gripper = m_World.GetActuator(Named(GripperActuator));
			uint finger = m_World.GetJointID(Named("joint7"));
			uint secondFinger = m_World.GetJointID(Named("joint8"));
            uint gripperActuator = m_World.GetActuatorIndex(Named(GripperActuator));
            uint expectedGripper = (uint)(ActuatorOffset + 6);
            if (m_Gripper == null || !TubeHomeMotion.CanClaimNamedGripper(Named(GripperActuator), JointPrefix, gripperActuator, expectedGripper,
                    gripperActuator == expectedGripper ? m_World.GetActuatorTrnJointID(gripperActuator) : -1, finger, secondFinger,
                    gripperActuator == expectedGripper && m_World.IsActuatorClaimedByPolicy(gripperActuator), m_Gripper.IK, m_GripperScriptOwned)
                || m_World.GetJointType(finger) != 2 || m_World.GetJointType(secondFinger) != 2
                || !m_World.GetJointRange(finger, out float minimum, out float maximum)
                || !m_World.GetJointRange(secondFinger, out float otherMinimum, out float otherMaximum)
                || !m_World.GetActuatorCtrlRange(gripperActuator, out float controlMinimum, out float controlMaximum)
                || !float.IsFinite(minimum) || !float.IsFinite(maximum) || !float.IsFinite(otherMinimum) || !float.IsFinite(otherMaximum)
                || !float.IsFinite(controlMinimum) || !float.IsFinite(controlMaximum)
                || minimum >= maximum || maximum > 0.000001f || otherMinimum < -0.000001f || otherMinimum >= otherMaximum
                || minimum < -0.050001f || otherMaximum > 0.050001f
                || MathF.Abs(minimum + otherMaximum) > 0.000001f || MathF.Abs(maximum + otherMinimum) > 0.000001f
                || MathF.Abs(controlMinimum - minimum) > 0.000001f || MathF.Abs(controlMaximum - maximum) > 0.000001f)
			{
                Abort("Cannot home: Pika must use the qualified mirrored slides and named position actuator, without another script or policy owner.");
				return;
			}
			m_MaximumOpening = -2000f * minimum;
			m_HomeFirstFingerAddress = m_World.GetJointQposAdr(finger);
			m_HomeSecondFingerAddress = m_World.GetJointQposAdr(secondFinger);
			m_HomeTargetGripper = (OpenMillimetres - m_MaximumOpening) / 2000f;
			m_HomeStartGripper = (float)m_Gripper.Value;
            if (!float.IsFinite(m_HomeStartGripper) || m_HomeStartGripper < minimum || m_HomeStartGripper > maximum
                || m_HomeTargetGripper < minimum || m_HomeTargetGripper > maximum
				|| m_HomeFirstFingerAddress == uint.MaxValue || m_HomeSecondFingerAddress == uint.MaxValue)
			{
				Abort("Cannot home: the open-jaw command is outside the Pika travel.");
				return;
			}
			// Keep the smooth jaw ramp no faster than the existing linear 0.6-second opening step.
			float jawRate = (OpenMillimetres - CloseMillimetres) / 0.6f;
			float jawDuration = 1.875f * 2000f * MathF.Abs(m_HomeTargetGripper - m_HomeStartGripper) / jawRate;
			m_HomeDuration = MathF.Max(jawDuration, TubeHomeMotion.Duration(m_HomeStart, AngularSpeed, DurationScale));
			m_HomeElapsed = 0f;
			m_HomeStable = 0f;
			m_HomeControlRevision = m_Task.ControlRevision;
			foreach (MujocoActuator? actuator in m_Arm) actuator!.IK = false;
			m_ArmScriptOwned = true;
			m_Gripper.IK = false;
            m_GripperScriptOwned = true;
			m_HomeReady = false;
			m_Homing = true;
			Status = TrialStatus("Returning all six simulated joints to zero, with jaws open. Recording continues until the arm is still.");
		}

		private bool AdvanceHoming(float deltaTime)
		{
			if (m_Task == null || m_Task.ControlRevision != m_HomeControlRevision || m_Task.PreparedPhysicsWorldEntityId != PhysicsWorldId
				|| (m_Task.EpisodeStage != TubeEpisodeStage.Ready && m_Task.EpisodeStage != TubeEpisodeStage.Active)
				|| (m_PlayStartup.CompletedEpisodes > 0 && !m_Task.RecordingCanContinue)
				|| !float.IsFinite(deltaTime) || deltaTime <= 0f)
			{
				Abort("Homing stopped because the task, control revision or simulation timing changed.");
				return false;
			}
			m_HomeElapsed += deltaTime;
			float fraction = TubeHomeMotion.Fraction(m_HomeElapsed / m_HomeDuration);
			bool settled = m_HomeElapsed >= m_HomeDuration;
			float tolerance = HomeToleranceDegrees * (MathF.PI / 180f);
			for (int i = 0; i < m_Arm.Length; i++)
			{
				m_Arm[i]!.Value = m_HomeStart[i] * (1f - fraction);
				float angle = m_World!.GetJointPosition(m_HomePositionAddresses[i]);
				float speed = m_World.GetJointVelocity(m_HomeVelocityAddresses[i]);
				settled &= float.IsFinite(angle) && float.IsFinite(speed) && MathF.Abs(angle) <= tolerance && MathF.Abs(speed) <= 0.05f;
			}
			m_Gripper!.Value = m_HomeStartGripper + (m_HomeTargetGripper - m_HomeStartGripper) * fraction;
			float first = m_World!.GetJointPosition(m_HomeFirstFingerAddress);
			float second = m_World.GetJointPosition(m_HomeSecondFingerAddress);
			float opening = m_MaximumOpening + 1000f * (first - second);
			settled &= float.IsFinite(opening) && MathF.Abs(opening - OpenMillimetres) <= 1f;
			m_HomeStable = settled ? m_HomeStable + deltaTime : 0f;
			if (m_HomeStable >= 0.2f)
			{
				m_Homing = false;
				m_HomeReady = true;
				Status = TrialStatus("All six joints are at zero and still; jaws are open.");
				return true;
			}
			if (m_HomeElapsed > m_HomeDuration + ExtraArrivalSeconds)
			{
				Abort("The simulated arm did not settle at zero. Automatic episodes stopped; no new tube start was prepared.");
			}
			return false;
		}

		private void ReleaseArmToGraph()
		{
			if (!m_ArmScriptOwned) return;
			foreach (MujocoActuator? actuator in m_Arm) actuator!.IK = true;
			m_ArmScriptOwned = false;
		}

		private void AppendMove(Vector3 contactPosition, Vector3 rotation, float speed, string label, float? tolerance = null, bool applyGraspOffset = true)
		{
			Quaternion wristRotation = new Quaternion(rotation);
			Vector3 offset = m_GraphSiteOffsetInWrist + (applyGraspOffset ? m_GraspForwardOffsetInWrist : Vector3.Zero);
			Vector3 target = contactPosition + wristRotation * offset;
			// LuckyIK solves the configured site frame, which includes the Pika mount's fixed rotation relative to link6.
			Vector3 siteRotation = (wristRotation * m_GraphSiteRotationInWrist).EulerAngles;
			Vector3 deltaRotation = siteRotation - m_PlannedRotation;
			float angle = MathF.Max(MathF.Abs(Mathf.WrapToPi(deltaRotation.X)), MathF.Max(MathF.Abs(Mathf.WrapToPi(deltaRotation.Y)), MathF.Abs(Mathf.WrapToPi(deltaRotation.Z))));
			float duration = m_DurationScale * MathF.Max(0.2f, MathF.Max((target - m_PlannedPosition).Length() / speed, angle / AngularSpeed));
			m_Queue.Enqueue(new Step { Kind = StepKind.Move, Position = target, Rotation = siteRotation, Duration = duration,
				PositionTolerance = tolerance ?? m_PositionTolerance, Label = label, WristRotation = wristRotation,
				MotionSpeed = speed, AppliesGraspOffset = applyGraspOffset });
			m_PlannedPosition = target;
			m_PlannedRotation = siteRotation;
		}

		private bool RetargetRelayToMeasuredGrasp()
		{
			if (!TrySiteWorldPose(m_ContactSite, out Vector3 contact, out _) ||
				!TrySiteWorldPose(m_MotionSite, out Vector3 plannedPosition, out Quaternion plannedOrientation) ||
				!TubeGraspGeometry.TryMeasuredReference(contact, m_Task!.PreparedTubeCentre, m_World!.GetOrientation(m_Wrist),
					m_GraspForwardOffsetInWrist, out Vector3 measuredReference)) return false;
			Vector3 correction = measuredReference - m_GraspForwardOffsetInWrist;
			Step[] remaining = m_Queue.ToArray();
			// Build the complete replacement first. If a target is invalid, the original
			// queue is not partly changed and the caller takes the normal hard-abort path.
			for (int i = 0; i < remaining.Length; i++)
			{
				Step step = remaining[i];
				if (step.Kind != StepKind.Move) continue;
				if (step.AppliesGraspOffset)
					step.Position = TubeGraspGeometry.RetargetSite(step.Position, step.WristRotation, correction,
						new Vector3(0f, GraspOffsetY, 0f));
				if (!Finite(step.Position) || !float.IsFinite(step.MotionSpeed) || step.MotionSpeed <= 0f) return false;
				Vector3 deltaRotation = step.Rotation - plannedOrientation.EulerAngles;
				float angle = MathF.Max(MathF.Abs(Mathf.WrapToPi(deltaRotation.X)),
					MathF.Max(MathF.Abs(Mathf.WrapToPi(deltaRotation.Y)), MathF.Abs(Mathf.WrapToPi(deltaRotation.Z))));
				float revisedDuration = m_DurationScale * MathF.Max(0.2f,
					MathF.Max((step.Position - plannedPosition).Length() / step.MotionSpeed, angle / AngularSpeed));
				if (!float.IsFinite(revisedDuration)) return false;
				step.Duration = MathF.Max(step.Duration, revisedDuration);
				remaining[i] = step;
				plannedPosition = step.Position;
				plannedOrientation = new Quaternion(step.Rotation);
			}
			m_Queue.Clear();
			foreach (Step step in remaining) m_Queue.Enqueue(step);
			m_GraspForwardOffsetInWrist = measuredReference;
			MeasuredGraspReferenceInWrist = measuredReference;
			MeasuredGraspCorrectionMm = correction.Length() * 1000f;
			m_PlannedPosition = plannedPosition;
			m_PlannedRotation = plannedOrientation.EulerAngles;
			return true;
		}

		private void AppendGripper(float opening, float seconds, string label)
		{
			m_Queue.Enqueue(new Step { Kind = StepKind.Gripper, OpeningMm = opening, Duration = seconds, Label = label });
		}

		private bool TrySiteWorldPose(uint site, out Vector3 position, out Quaternion orientation)
		{
			position = Vector3.Zero;
			orientation = Quaternion.Identity;
			if (!m_World!.GetSitePoseMj(site, out Vector3 nativePosition, out Quaternion nativeOrientation, out _))
			{
				return false;
			}
			Quaternion axis = m_World.GetAxisHzToMj().Conjugate;
			position = axis * nativePosition + m_World.GetSceneTranslation();
			orientation = axis * nativeOrientation;
			return true;
		}

		private Vector3 ApproachRotation(Vector3 contactPosition)
		{
			if (!RadialSideGrasp)
			{
				return GraspOrientationDegrees * (MathF.PI / 180f);
			}
			Quaternion axis = m_World!.GetAxisHzToMj();
			Vector3 modelPosition = axis * (contactPosition - m_World.GetSceneTranslation());
			if (JointPrefix.Length > 0)
			{
				uint baseBody = m_World.GetBodyID(Named("base_link"));
				if (baseBody != uint.MaxValue) modelPosition -= m_World.GetPosition(baseBody, hazelSpace: false);
			}
			float heading = MathF.Atan2(modelPosition.Y, modelPosition.X);
			float pitch = RadialApproachPitchDegrees * (MathF.PI / 180f);
			Quaternion modelRotation = new Quaternion(new Vector3(0f, 0f, heading)) * new Quaternion(new Vector3(0f, MathF.PI * 0.5f + pitch, 0f));
			return (axis.Conjugate * modelRotation).EulerAngles;
		}

		private void Send(Vector3 position, Vector3 rotation, float duration)
		{
			if (m_RelayManaged && m_Task?.TwoRobotRelay == true) m_Robot!.SetInputBool(new Identifier("ReturnArm"), JointPrefix.Length > 0);
			m_Robot!.SetInputVector3(MotionInputPrefix.Length == 0 ? k_Position : new Identifier(MotionInputPrefix + "Target_Position"), position);
			m_Robot.SetInputVector3(MotionInputPrefix.Length == 0 ? k_Rotation : new Identifier(MotionInputPrefix + "Target_Orientation"), rotation * (180f / MathF.PI));
			m_Robot.SetInputFloat(MotionInputPrefix.Length == 0 ? k_Duration : new Identifier(MotionInputPrefix + "Duration"), duration);
			m_Robot.SetInputTrigger(MotionInputPrefix.Length == 0 ? k_Start : new Identifier(MotionInputPrefix + "Start"));
		}

		private void CompleteStep()
		{
			m_Queue.Dequeue();
			m_Started = false;
		}

		private void RecordTubeDiagnostic(int stage)
		{
			if (m_Task == null) return;
			Vector3 point = m_Task.PreparedTubeCentre;
			Vector3 goal = m_RelayManaged ? m_RelayGoal : m_Task.PreparedGoalCentre;
			if (!Finite(point) || !Finite(goal)) return;
			float dx = point.X - goal.X, dz = point.Z - goal.Z;
			float errorMm = MathF.Sqrt(dx * dx + dz * dz) * 1000f;
			switch (stage)
			{
				case 0: TubeBeforeRelease = point; BeforeReleaseErrorMm = errorMm; break;
				case 1: TubeAfterRelease = point; AfterReleaseErrorMm = errorMm; break;
				case 2: TubeAfterReleaseSettled = point; AfterReleaseSettledErrorMm = errorMm; break;
				case 3: TubeAfterWithdraw = point; AfterWithdrawErrorMm = errorMm; break;
				case 4: TubeAtPlacementCheck = point; PlacementCheckErrorMm = errorMm; break;
			}
		}

		private void RecordExpectedMiss(string reason)
		{
			m_TrialOutcome.RecordExpectedMiss(reason);
			LastGraspFailure = m_TrialOutcome.Reason;
			// Continue the original validated path through release, withdrawal and Home.
			// A simulated miss does not prove the physical gripper is empty.
			m_PlayStartup.Cancel();
			Status = TrialStatus("Continuing the planned path and returning to zero; this trial will be unsuccessful.");
		}

		private string TrialStatus(string action)
		{
			return m_TrialOutcome.HasExpectedMiss ? $"Missed grasp: {m_TrialOutcome.Reason} {action}" : action;
		}

		private void Abort(string reason)
		{
			bool wasBusy = m_Busy || m_Homing;
			m_Homing = false;
			m_HomeReady = false;
			m_PlayStartup.Cancel();
			m_Busy = false;
			m_Started = false;
			m_Queue.Clear();
			if (wasBusy && m_Robot != null && m_World != null)
			{
				if (m_ArmScriptOwned)
				{
					for (int i = 0; i < m_Arm.Length; i++)
					{
						float position = m_World.GetJointPosition(m_HomePositionAddresses[i]);
						if (float.IsFinite(position)) m_Arm[i]!.Value = position;
					}
				}
				else if (TrySiteWorldPose(m_MotionSite, out Vector3 position, out Quaternion rotation))
				{
					Send(position, rotation.EulerAngles, 0.2f);
				}
				if (!m_RelayManaged && m_Task != null && m_Task.EpisodeActive)
				{
					m_Task.FinishUnsuccessfully();
				}
			}
			if (m_RelayManaged)
			{
				RelayLegComplete = true;
				RelayLegSuccessful = false;
				RelayLegFaulted = true;
			}
			Status = m_TrialOutcome.HasExpectedMiss ? $"{reason} Earlier task warning: {m_TrialOutcome.Reason}" : reason;
		}

		private bool ValidSettings()
		{
			float[] values = { TravelSpeed, VerticalSpeed, AngularSpeed, DurationScale, HoverHeight, PositionTolerance, GraspPositionTolerance, OrientationToleranceRadians, ExtraArrivalSeconds };
			foreach (float value in values)
			{
				if (!float.IsFinite(value) || value <= 0f)
				{
					return false;
				}
			}
			return DurationScale >= 1f && MaximumEpisodes >= 0 && ActuatorOffset >= 0 && JointPrefix != null && MotionInputPrefix != null
				&& float.IsFinite(HomeToleranceDegrees) && HomeToleranceDegrees > 0f && HomeToleranceDegrees <= 2f
				&& float.IsFinite(RadialApproachPitchDegrees) && MathF.Abs(RadialApproachPitchDegrees) <= 45f
				&& float.IsFinite(GraspForwardOffsetMm) && GraspForwardOffsetMm >= 0f && GraspForwardOffsetMm <= 50f
				&& float.IsFinite(GraspOffsetY) && float.IsFinite(GraspOrientationDegrees.X) && float.IsFinite(GraspOrientationDegrees.Y) && float.IsFinite(GraspOrientationDegrees.Z)
				&& OpenMillimetres <= 90f && OpenMillimetres > 43f && CloseMillimetres >= 0f && CloseMillimetres < 43f;
		}
	}

	internal enum TubePlayCommand { None, Prepare, Begin, Run, Home }
	internal enum TubeTrialResult { NotReady, Successful, Unsuccessful }

	internal sealed class TubeTrialOutcome
	{
		public string Reason { get; private set; } = "";
		public bool HasExpectedMiss => Reason.Length != 0;
		public void Reset() { Reason = ""; }
		public void RecordExpectedMiss(string reason)
		{
			if (!HasExpectedMiss && !string.IsNullOrWhiteSpace(reason)) Reason = reason;
		}
		public string PhaseKind(string kind) => HasExpectedMiss ? "missed-grasp-" + kind : kind;
		public TubeTrialResult Resolve(bool homeReady, bool placementComplete)
		{
			if (!homeReady) return TubeTrialResult.NotReady;
			if (HasExpectedMiss) return TubeTrialResult.Unsuccessful;
			return placementComplete ? TubeTrialResult.Successful : TubeTrialResult.NotReady;
		}
	}

	internal static class TubeGraspGeometry
	{
		public static bool TryMeasuredReference(Vector3 contactWorld, Vector3 objectWorld, Quaternion wristRotation,
			Vector3 plannedReference, out Vector3 measuredReference)
		{
			measuredReference = Vector3.Zero;
			if (!Finite(contactWorld) || !Finite(objectWorld) || !Finite(plannedReference) ||
				!float.IsFinite(wristRotation.X) || !float.IsFinite(wristRotation.Y) ||
				!float.IsFinite(wristRotation.Z) || !float.IsFinite(wristRotation.W)) return false;
			float norm = wristRotation.X * wristRotation.X + wristRotation.Y * wristRotation.Y +
				wristRotation.Z * wristRotation.Z + wristRotation.W * wristRotation.W;
			if (MathF.Abs(norm - 1f) > 0.001f) return false;
			measuredReference = wristRotation.Conjugate * (contactWorld - objectWorld);
			// A small, measured grasp-depth/centering correction is allowed. A remote
			// object or inconsistent transform must not turn a failed grasp into a target.
			return Finite(measuredReference) && measuredReference.Length() <= 0.100f &&
				(measuredReference - plannedReference).Length() <= 0.050f;
		}

		public static Vector3 RetargetSite(Vector3 originalSiteTarget, Quaternion targetWristRotation,
			Vector3 correctionInWrist, Vector3 nominalContactOffset) =>
			originalSiteTarget + targetWristRotation * correctionInWrist - nominalContactOffset;

		private static bool Finite(Vector3 point) => float.IsFinite(point.X) && float.IsFinite(point.Y) && float.IsFinite(point.Z);

		public static bool TryForwardOffsetInWrist(Quaternion graspWristRotation, float offsetMm, out Vector3 offsetInWrist)
		{
			offsetInWrist = Vector3.Zero;
			if (!float.IsFinite(offsetMm) || offsetMm < 0f || offsetMm > 50f)
			{
				return false;
			}
			if (offsetMm == 0f)
			{
				return true;
			}
			// Pika finger +X is link6 wrist +Z. Project once at the original pickup
			// orientation, then keep that reference fixed in the tool through the carry.
			Vector3 forward = graspWristRotation * new Vector3(0f, 0f, 1f);
			float length = MathF.Sqrt(forward.X * forward.X + forward.Z * forward.Z);
			if (!float.IsFinite(length) || length < 0.0001f)
			{
				return false;
			}
			Vector3 worldOffset = new Vector3(forward.X, 0f, forward.Z) * (offsetMm * 0.001f / length);
			offsetInWrist = graspWristRotation.Conjugate * worldOffset;
			return float.IsFinite(offsetInWrist.X) && float.IsFinite(offsetInWrist.Y) && float.IsFinite(offsetInWrist.Z);
		}

		public static Vector3 ObjectPointFromContact(Vector3 contact, Quaternion wristRotation, Vector3 offsetInWrist)
		{
			return contact - wristRotation * offsetInWrist;
		}
	}

	internal static class TubeGraspCheck
	{
		// A tube must remain lifted, between the fingers and measurably holding them apart.
		// This is a simulated grasp check, not evidence of a physical grasp.
		public static bool IsHeld(float tubeY, float originalY, float openingMm, float closeTargetMm, float separation, float maximumSeparation)
		{
			return float.IsFinite(tubeY) && float.IsFinite(originalY) && tubeY >= originalY + 0.040f
				&& float.IsFinite(openingMm) && float.IsFinite(closeTargetMm) && openingMm > closeTargetMm + 1f
				&& float.IsFinite(separation) && float.IsFinite(maximumSeparation) && maximumSeparation > 0f
				&& separation <= maximumSeparation;
		}
	}

	internal static class TubeHomeMotion
	{
        public static bool CanClaimNamedGripper(string actuatorName, string prefix, uint actuatorIndex, uint expectedIndex,
            int transmissionJoint, uint firstJoint, uint secondJoint, bool policyOwned, bool graphOwned, bool ownedByThisDriver) =>
            actuatorName == prefix + "gripper" && actuatorIndex == expectedIndex && firstJoint != uint.MaxValue && secondJoint != uint.MaxValue &&
            firstJoint != secondJoint && transmissionJoint == (int)firstJoint && !policyOwned && (graphOwned || ownedByThisDriver);

        public static bool CanClaimGripper(string actuatorName, uint actuatorIndex, int transmissionJoint,
            uint firstJoint, uint secondJoint, bool policyOwned, bool graphOwned, bool ownedByThisDriver) =>
            actuatorName == "gripper" && actuatorIndex == 6 && firstJoint != uint.MaxValue && secondJoint != uint.MaxValue &&
            firstJoint != secondJoint && transmissionJoint == (int)firstJoint && !policyOwned && (graphOwned || ownedByThisDriver);

		public static float Fraction(float fraction)
		{
			if (fraction <= 0f)
			{
				return 0f;
			}
			if (fraction >= 1f)
			{
				return 1f;
			}
			// Reflect the second half so cancellation near t=1 cannot produce
			// a value above 1 and command a joint past its zero limit.
			double t = fraction;
			bool reflected = t > 0.5;
			if (reflected)
			{
				t = 1.0 - t;
			}
			double smooth = t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
			return Math.Clamp((float)(reflected ? 1.0 - smooth : smooth), 0f, 1f);
		}

		public static float Duration(float[] controls, float radiansPerSecond, float scale)
		{
			float distance = 0f;
			foreach (float control in controls) distance = MathF.Max(distance, MathF.Abs(control));
			// Quintic interpolation's peak derivative is 1.875, so the requested joint rate is bounded.
			return MathF.Max(0.2f, distance * 1.875f / radiansPerSecond) * scale;
		}
	}

	// The task owns qualification, physics and finalization. This controller may prepare another
	// start only after its own successful episode has returned home and reached genuine Complete.
	internal sealed class TubePlayStartup
	{
		private enum Phase { WaitingForTask, WaitingForReady, Homing, WaitingForPlacement, WaitingForActive, Running, WaitingForFinalization, WaitingForFollower, Finished }
		private Phase m_Phase;
		private float m_Elapsed;
		private ulong m_TaskId;
		private int m_Revision;
		private bool m_RunIssued;
		public bool Finished => m_Phase == Phase.Finished;
		public int CompletedEpisodes { get; private set; }
		public string Status { get; private set; } = "Starting simulated pick/place/home.";

		public void Cancel() { m_Phase = Phase.Finished; m_RunIssued = false; }
		public void AcceptOwnControlRevision(int revision) { m_Revision = revision; }

		public void EpisodeFinished(int revision)
		{
			if (!m_RunIssued || (m_Phase != Phase.Running && m_Phase != Phase.Finished)) return;
			m_RunIssued = false;
			m_Revision = revision;
			m_Elapsed = 0f;
			m_Phase = Phase.WaitingForFinalization;
		}

		public TubePlayCommand Advance(bool enabled, bool taskAvailable, bool simulationOnly, TubeEpisodeStage stage,
			ulong taskId, int revision, string taskStatus, float deltaTime, bool homeReady = true, bool continuous = false,
			bool episodeSuccessful = false, bool recordingCanContinue = true, bool waitForPlacement = false,
			bool placementConfirmed = true, int maximumEpisodes = 0,
			Hazel.Data.EpisodeFollowerStatus followerStatus = Hazel.Data.EpisodeFollowerStatus.NotRequired, bool followerCanAdvance = true)
		{
			if (Finished) return TubePlayCommand.None;
			if (!enabled) return Stop("Automatic episodes are off. Prepare and begin an episode, then use Run simulated demonstration.");
			if (maximumEpisodes < 0) return Stop("MaximumEpisodes must be zero (unlimited) or a positive count.");
			m_Elapsed += float.IsFinite(deltaTime) ? MathF.Max(0f, deltaTime) : 0f;
			if (m_Phase == Phase.WaitingForTask)
			{
				if (!taskAvailable || stage == TubeEpisodeStage.WaitingForPlay)
				{
					Status = "Waiting for the episode controls to initialize.";
					return m_Elapsed < 3f ? TubePlayCommand.None : Stop("Automatic episodes could not start: " + taskStatus);
				}
				if (!simulationOnly) return Stop("Automatic episodes are simulation-only. Select SimulationOnly, then restart Play, or use the manual controls.");
				if (revision != 0 || stage != TubeEpisodeStage.Idle) return Stop("Automatic episodes left the task unchanged because it was already used: " + taskStatus);
				m_TaskId = taskId;
				m_Revision = revision;
				return Prepare();
			}
			if (!taskAvailable || taskId != m_TaskId) return Stop("Automatic episodes stopped: the assigned episode controls changed or were removed.");
			if (!simulationOnly) return Stop("Automatic episodes stopped: the task is no longer SimulationOnly.");
			if (revision != m_Revision) return Stop("Automatic episodes cancelled by a manual task action. Use the manual controls or restart Play.");
			if (stage == TubeEpisodeStage.Failed) return Stop("Automatic episodes stopped: " + taskStatus);
			if (m_Phase == Phase.Running)
			{
				return stage == TubeEpisodeStage.Active ? TubePlayCommand.None : Stop("The running episode ended outside the demonstration: " + taskStatus);
			}
			if (m_Phase == Phase.WaitingForFinalization)
			{
				if (stage == TubeEpisodeStage.Complete)
				{
					if (!episodeSuccessful) return Stop("Episode finalization was unsuccessful. Automatic episodes stopped: " + taskStatus);
					CompletedEpisodes++;
					m_Phase = Phase.WaitingForFollower;
				}
				else
				{
					if (stage != TubeEpisodeStage.Finalizing) return Stop("Episode did not enter finalization: " + taskStatus);
					Status = "Arm at zero. Waiting for recording and trajectory export to finish before randomizing the next tube.";
					return m_Elapsed <= 30f ? TubePlayCommand.None : Stop("Episode finalization timed out; nothing was reset: " + taskStatus);
				}
			}
			if (m_Phase == Phase.WaitingForFollower)
			{
				if (stage != TubeEpisodeStage.Complete) return Stop("The completed episode changed while waiting for its physical follower.");
				if (followerStatus == Hazel.Data.EpisodeFollowerStatus.Stopped)
					return Stop("The physical follower stopped. The tube remains at this episode's goal; stop Play before starting another linked run.");
				if (!followerCanAdvance)
				{
					Status = "Simulation finished. Waiting for the physical robot to finish this episode; the next tube start is held.";
					return TubePlayCommand.None;
				}
				if (!continuous || (maximumEpisodes > 0 && CompletedEpisodes >= maximumEpisodes))
					return Stop($"Finished {CompletedEpisodes} episode(s). All registered followers have completed.");
				if (!recordingCanContinue) return Stop("Native recording stopped or reached its episode limit. Automatic episodes stopped.");
				return Prepare();
			}
			if (CompletedEpisodes > 0 && !recordingCanContinue)
				return Stop("Native recording stopped between episodes. Automatic episodes stopped; the recording session will not be restarted.");
			if (m_Phase == Phase.WaitingForPlacement)
			{
				if (stage != TubeEpisodeStage.Ready || !homeReady) return Stop("The prepared start changed while waiting for the real tube.");
				if (waitForPlacement && !placementConfirmed)
				{
					Status = "Arm at zero. Place the REAL tube at the displayed start mark, then click 'Real tube is at the displayed start — continue'. Only the simulated tube was randomized.";
					return TubePlayCommand.None;
				}
				return Begin();
			}
			if (m_Elapsed > 30f) return Stop("Automatic episodes timed out while waiting: " + taskStatus);
			if (m_Phase == Phase.WaitingForReady || m_Phase == Phase.Homing)
			{
				if (stage == TubeEpisodeStage.Ready)
				{
					if (!homeReady)
					{
						Status = "Returning all six simulated joints to zero before opening the episode.";
						if (m_Phase == Phase.Homing) return TubePlayCommand.None;
						m_Phase = Phase.Homing;
						m_Elapsed = 0f;
						return TubePlayCommand.Home;
					}
					if (waitForPlacement && !placementConfirmed)
					{
						m_Phase = Phase.WaitingForPlacement;
						m_Elapsed = 0f;
						Status = "Arm at zero. Place the real tube at the displayed start and confirm to begin.";
						return TubePlayCommand.None;
					}
					return Begin();
				}
				if (m_Phase == Phase.Homing || stage != TubeEpisodeStage.Preparing) return Stop("Automatic episodes could not prepare: " + taskStatus);
				Status = "Preparing the tube: " + taskStatus;
				return TubePlayCommand.None;
			}
			if (stage == TubeEpisodeStage.Active)
			{
				m_Phase = continuous ? Phase.Running : Phase.Finished;
				m_RunIssued = true;
				Status = "Episode ready. Starting simulated pick/place/home.";
				return TubePlayCommand.Run;
			}
			if (stage != TubeEpisodeStage.Beginning) return Stop("Automatic episodes could not begin: " + taskStatus);
			Status = "Beginning the episode: " + taskStatus;
			return TubePlayCommand.None;
		}

		private TubePlayCommand Prepare()
		{
			m_Phase = Phase.WaitingForReady;
			m_Elapsed = 0f;
			Status = $"Preparing random tube start for episode {CompletedEpisodes + 1}.";
			return TubePlayCommand.Prepare;
		}

		private TubePlayCommand Begin()
		{
			m_Phase = Phase.WaitingForActive;
			m_Elapsed = 0f;
			Status = "Arm at zero and tube start ready. Saving the start and opening the episode.";
			return TubePlayCommand.Begin;
		}

		private TubePlayCommand Stop(string status)
		{
			Status = status;
			Cancel();
			return TubePlayCommand.None;
		}
	}
}
