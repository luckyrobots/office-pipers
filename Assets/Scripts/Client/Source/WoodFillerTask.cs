using System;
using System.IO;
using System.Threading.Tasks;
using Hazel;
using Hazel.Data;

namespace OfficePipers
{
	public enum TubeWorkflow { RealDemonstration, SimulationOnly }
	public enum TubeEpisodeStage { WaitingForPlay, Idle, Preparing, Ready, Beginning, Active, Complete, Failed, Finalizing }

	public class WoodFillerTask : Entity
	{
		[Group("Task")]
		public TubeWorkflow Workflow = TubeWorkflow.SimulationOnly;
		[ReadOnly] public string Status = "Prepare a start. This script runs in Play and never commands the real robot.";
		[ReadOnly] public string StartPosition = "Not prepared";
		[ReadOnly] public string GoalPosition = "Desk cross: LEFT 903 mm / BACK 350 mm";
		[ReadOnly] public int EpisodeNumber;
		[ReadOnly] public bool SimulatedPlacementComplete;
		[ReadOnly] public string PlacementCheck = "Real object pose is not tracked by the simulation check.";
		[ReadOnly] public float PlacementErrorMm, PlacementHeightErrorMm, PlacementTiltDegrees;
		[ReadOnly] public float PlacementSpeedMmPerSecond, PlacementAngularSpeedDegrees;

		[Group("Scene references")]
		[Tooltip("MuJoCo scene containing the free tube body. Robot and tube must share this instance for simulated grasp contacts.")]
		public Entity? PhysicsWorld;
		public string TubeBodyName = "wood_filler_tube";
		[Tooltip("Optional static-mesh template. Only its mesh/scale/rotation are copied into a new visual-only runtime marker; the template is never moved.")]
		public Entity? StartMarkerTemplate;
		public Entity? ActiveRobotBase;
		public Entity? OtherRobotBase1;
		public Entity? OtherRobotBase2;
		public Entity? OtherRobotBase3;

		[Group("Desk and tube (metres)")]
		public float DeskWidth = 1.4f;
		public float DeskDepth = 0.7f;
		public float DeskSurfaceY = 0.75f;
		public float TubeHeight = 0.193f;
		public float TubeDiameter = 0.043f;
		[Tooltip("Estimated until the physical tube has been weighed.")]
		public float TubeMassKg = 0.180f;
		public float GoalLeft = 0.903f;
		public float GoalBack = 0.350f;

		[Group("Fixed start (metres / degrees)")]
		[Tooltip("Use this measured start and yaw for every episode. Desk-edge, robot-base, reach and place-target clearances still apply.")]
		public bool UseFixedStart;
		public float FixedStartLeft = 0.900f;
		public float FixedStartBack = 0.120f;
		public float FixedStartYawDegrees = 180f;

		[Group("Seeded start region (metres)")]
		public int Seed = 15093;
		public float LeftMin = 0.950f;
		public float LeftMax = 0.980f;
		public float BackMin = 0.160f;
		public float BackMax = 0.220f;
		public float EdgeClearance = 0.060f;
		public float GoalExclusionRadius = 0.120f;
		[Tooltip("Planar reach bound only; this does not prove IK reachability or a collision-free robot path.")]
		public float MaximumBaseDistance = 0.700f;
		public float BaseExclusionRadius = 0.180f;

		[Group("Simulated placement check")]
		public float SettleSeconds = 0.5f;
		public float MinimumLift = 0.030f;
		public float GoalTolerance = 0.025f;
		public float StableSeconds = 0.5f;
		public float HeightTolerance = 0.008f;
		public float MaximumRestSpeed = 0.015f;
		public float MaximumAngularSpeed = 0.15f;

		[Group("Native virtual recording (optional)")]
		[Tooltip("Records configured VIRTUAL scene streams only. It does not capture the real cameras or real robot.")]
		public bool RecordVirtualScene;
		public int TaskIndex;
		[Group("Trajectory export")]
		public bool ExportTrajectory = true;
		[Tooltip("Exact source XML of the combined task world. SHA-256 and its authored position-actuator tags are checked on a worker before preparation; changed assets must be requalified.")]
		public string PositionActuatorModelSource = "";
		[Tooltip("Independent project-owned export directory; native Recorder output paths are not guessed.")]
		public string ExportDirectory = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "LuckyRobots", "OfficePipers", "TubeEpisodes");
		[Tooltip("Optional exact native Recorder session label, copied as an operator-supplied label; leave blank if not configured.")]
		public string NativeSessionLabel = "";
		[ReadOnly] public string ExportedArtifact = "Not exported";
		[ReadOnly] public float MaximumFingerAsymmetryMm;

		private enum State { Idle, QualifyingSource, ResetQueued, Settling, Ready, BeginQueued, PublishingStart, ValidationBeginQueued, Active, EndQueued, Exporting, Complete, Error }
		private State m_State;
		private MujocoSceneComponent? m_World;
		private uint m_TubeBody;
		private Vector3 m_Start;
		private float m_Yaw;
		private float m_Elapsed;
		private float m_StableElapsed;
		private float m_PlacementDiagnosticElapsed;
		private bool m_LiftObserved;
		private bool m_EndSuccess;
		private bool m_OwnsEpisode;
		private bool m_RecordEpisode;
		private TubeWorkflow m_Workflow;
		private ulong m_ResetSync;
		private readonly TubeSpawnSettings m_Spawn = new TubeSpawnSettings();
		private readonly WoodFillerTrajectory m_Trajectory = new WoodFillerTrajectory();
		private readonly string m_SessionId = Guid.NewGuid().ToString("N");
		private TubeEpisodeExport? m_Artifact;
		private Task<string>? m_ExportJob;
		private Task<string>? m_StartJob;
		private Task<TubeActuatorQualification>? m_QualificationJob;
		private TubeActuatorQualification m_Qualification = new();
		private bool m_ExportThisEpisode;
		private bool m_CaptureValid;
		private string m_CaptureError = "";
		private string m_ExportDirectory = "";
		private bool m_OwnsNativeSession;
		private int m_NativeOrdinal;
		private ulong m_WorldEntityId;
		private int m_PreparedSeed, m_PreparedOrdinal, m_PreparedTaskIndex;
		private float m_DeskY, m_TubeHeight, m_TubeDiameter, m_TubeMass;
		private float m_SettleSeconds, m_MinimumLift, m_GoalTolerance, m_StableSeconds, m_HeightTolerance, m_MaximumRestSpeed, m_MaximumAngularSpeed;
		private Entity? m_StartMarker;
		private bool m_RuntimeCreated;
		private int m_BeginControlRevision;
		private string m_EpisodeId = "";

		public bool EpisodeActive => m_State == State.Active;
		public bool EpisodeSucceeded => m_State == State.Complete && m_EndSuccess && (!m_ExportThisEpisode || m_CaptureValid);
		public bool RecordingCanContinue => !m_RecordEpisode || (m_OwnsNativeSession && Observer.IsRecording);
		internal bool PreparedSourceQualified => m_Qualification.Verified;
		public int ControlRevision { get; private set; }
		public TubeEpisodeStage EpisodeStage => !m_RuntimeCreated ? TubeEpisodeStage.WaitingForPlay : m_State switch
		{
			State.Idle => TubeEpisodeStage.Idle,
			State.QualifyingSource or State.ResetQueued or State.Settling => TubeEpisodeStage.Preparing,
			State.Ready => TubeEpisodeStage.Ready,
			State.BeginQueued or State.PublishingStart or State.ValidationBeginQueued => TubeEpisodeStage.Beginning,
			State.Active => TubeEpisodeStage.Active,
			State.EndQueued or State.Exporting => TubeEpisodeStage.Finalizing,
			State.Error => TubeEpisodeStage.Failed,
			State.Complete => TubeEpisodeStage.Complete,
			_ => TubeEpisodeStage.Failed
		};
		public Vector3 PreparedTubeCentre => m_World == null ? m_Start : m_World.GetPosition(m_TubeBody) + m_World.GetSceneTranslation();
		public Vector3 PreparedGoalCentre => new Vector3(m_Spawn.GoalLeft - m_Spawn.Width * 0.5f, m_DeskY + m_TubeHeight * 0.5f, m_Spawn.GoalBack - m_Spawn.Depth * 0.5f);
		public float PreparedYawRadians => m_Yaw;
		public ulong PreparedPhysicsWorldEntityId => m_WorldEntityId;
		public EpisodeFollowerStatus FollowerStatus => EpisodeFollowers.Status(m_WorldEntityId);
		public bool FollowerCanAdvance => EpisodeFollowers.CanAdvance(m_WorldEntityId);
		internal bool PublishPhase(string phase) => m_EpisodeId.Length != 0 && EpisodeFollowers.SetPhase(m_WorldEntityId, m_EpisodeId, phase);

		protected override void OnCreate()
		{
			m_RuntimeCreated = true;
			if (Observer.IsEpisodeOpen)
			{
				Fail("End the existing native episode before preparing a start. Nothing was reset.");
			}
		}

		[Button("1. Prepare start")]
		[Tooltip("Prepares the fixed start when enabled, otherwise samples the seeded region. Resets only the simulated tube, before any recording. Place the physical tube at the displayed mark yourself.")]
		public void PrepareNextEpisode()
		{
			ControlRevision++;
			if (!FollowerCanAdvance)
			{
				Status = FollowerStatus == EpisodeFollowerStatus.Stopped
					? "The physical follower stopped. Stop Play before starting a new linked run; the tube has not moved."
					: "Waiting for the physical robot to finish this episode before moving the tube to another start.";
				return;
			}
			if (Observer.IsEpisodeOpen || m_State == State.Active || m_State == State.EndQueued)
			{
				Status = "Finish the current episode before preparing another start. Nothing was reset.";
				return;
			}
			if (m_State == State.QualifyingSource || m_State == State.ResetQueued || m_State == State.Settling || m_State == State.BeginQueued || m_State == State.PublishingStart || m_State == State.ValidationBeginQueued || m_State == State.Exporting
				|| (m_ExportJob != null && !m_ExportJob.IsCompleted) || (m_StartJob != null && !m_StartJob.IsCompleted))
			{
				return;
			}
			if (!Configure())
			{
				return;
			}
			TubeStart sample;
			string error;
			bool startValid = UseFixedStart
				? WoodFillerLayout.TryFixedStart(m_Spawn, FixedStartLeft, FixedStartBack, FixedStartYawDegrees, out sample, out error)
				: WoodFillerLayout.TrySample(m_Spawn, Seed, EpisodeNumber, out sample, out error);
			if (!startValid)
			{
				Fail(error);
				return;
			}
			m_Start = new Vector3(sample.Left - DeskWidth * 0.5f, DeskSurfaceY + TubeHeight * 0.5f + 0.001f, sample.Back - DeskDepth * 0.5f);
			m_Yaw = sample.YawRadians;
			StartPosition = $"LEFT {sample.Left * 1000f:F0} mm / BACK {sample.Back * 1000f:F0} mm / yaw {sample.YawRadians * 180f / MathF.PI:F0} deg";
			GoalPosition = $"Desk cross: LEFT {GoalLeft * 1000f:F0} mm / BACK {GoalBack * 1000f:F0} mm";
			m_QualificationJob = TubeActuatorQualification.VerifyAsync(PositionActuatorModelSource);
			m_State = State.QualifyingSource;
			Status = "Checking the pinned task model and position-actuator recipe before preparing a start.";
		}

		[Button("2. Real tube is at start mark — begin")]
		public void ConfirmRealTubeAndBegin() { ControlRevision++; QueueBegin(TubeWorkflow.RealDemonstration); }

		[Button("2. Begin simulation-only episode")]
		public void BeginSimulationEpisode() { ControlRevision++; QueueBegin(TubeWorkflow.SimulationOnly); }

		// Only the exact automatic Begin that requested this transition may cancel it.
		// A different accepted Begin, other physics world or external recorder remains untouched.
		internal bool CancelAutomaticBegin(int expectedRevision, ulong expectedWorld, string reason)
		{
			if (m_BeginControlRevision != expectedRevision || m_WorldEntityId != expectedWorld || m_Workflow != TubeWorkflow.SimulationOnly)
			{
				return false;
			}
			if (m_State == State.BeginQueued || m_State == State.PublishingStart || m_State == State.ValidationBeginQueued)
			{
				ControlRevision++;
				// A manifest write may finish in the background, but this terminal state
				// prevents it from subsequently opening a native recording episode.
				Fail(reason);
				return true;
			}
			if (m_State == State.Active)
			{
				ControlRevision++;
				QueueEnd(false);
				Status = reason + " Finishing the automatically opened episode as unsuccessful.";
				return true;
			}
			return false;
		}

		[Button("3. Tube released at cross — finish successfully")]
		[Tooltip("Your explicit success label after releasing the upright tube. Real success is not sensor verified.")]
		public void FinishSuccessfully() { ControlRevision++; QueueEnd(true); }

		[Button("Finish as unsuccessful")]
		public void FinishUnsuccessfully() { ControlRevision++; QueueEnd(false); }

		[Button("Stop native virtual recording session")]
		public void StopVirtualRecording()
		{
			ControlRevision++;
			if (m_OwnsEpisode || Observer.IsEpisodeOpen)
			{
				Status = "Finish the episode first, then stop the recording session.";
				return;
			}
			Observer.StopRecording();
			m_OwnsNativeSession = false;
			m_NativeOrdinal = 0;
			Status = "Native virtual recording stopped. Real robot and cameras were not changed.";
		}

		protected override void OnUpdate(float deltaTime)
		{
			if ((m_State == State.QualifyingSource || m_State == State.Settling || m_State == State.Ready || m_State == State.BeginQueued || m_State == State.PublishingStart || m_State == State.ValidationBeginQueued || m_State == State.Active || m_State == State.EndQueued)
				&& Scene.FindEntityByHandle((EntityHandle)m_WorldEntityId) == null)
			{
				Fail("The prepared physics world was removed. This episode cannot continue.");
				return;
			}
			if (!m_OwnsEpisode && Observer.IsEpisodeOpen)
			{
				Fail("An episode opened outside this task. End it and use Prepare; no late randomization was performed.");
				return;
			}
			if (m_State == State.QualifyingSource && m_QualificationJob!.IsCompleted)
			{
				if (!m_QualificationJob.IsCompletedSuccessfully)
				{
					Fail("Task source qualification failed: " + m_QualificationJob.Exception!.GetBaseException().Message);
				}
				else
				{
					m_Qualification = m_QualificationJob.Result;
					m_State = State.ResetQueued;
					Status = "Qualified task source. Preparing the simulated start at the next sync point.";
				}
			}
			else if (m_State == State.ResetQueued && TimeSync.IsAtSyncPoint)
			{
				ResetTube();
			}
			else if (m_State == State.Settling)
			{
				m_Elapsed += deltaTime;
				Vector3 position = m_World!.GetPosition(m_TubeBody) + m_World.GetSceneTranslation();
				Vector3 up = m_World.GetOrientation(m_TubeBody) * new Vector3(0f, 0f, 1f);
				bool settled = (position - m_Start).Length() <= m_HeightTolerance * 2f && up.Y >= 0.98f
					&& m_World.GetVelocity(m_TubeBody).Length() <= m_MaximumRestSpeed
					&& m_World.GetAngularVelocity(m_TubeBody).Length() <= m_MaximumAngularSpeed;
				if (m_Elapsed >= m_SettleSeconds && settled && TimeSync.IsAtSyncPoint && TimeSync.CurrentSyncIndex != m_ResetSync)
				{
					m_State = State.Ready;
					Status = Workflow == TubeWorkflow.RealDemonstration
						? "Place the REAL tube upright at the displayed start, then confirm above. No robot motion is automatic."
						: "Start ready. Begin the simulation-only episode, then demonstrate pick and place.";
				}
				else if (m_Elapsed > m_SettleSeconds + 3f)
				{
					Fail("The simulated tube did not settle at its start. Check the desk collider and tube physics before beginning.");
				}
			}
			else if (m_State == State.BeginQueued && TimeSync.IsAtSyncPoint)
			{
				BeginEpisode();
			}
			else if (m_State == State.PublishingStart && m_StartJob!.IsCompleted)
			{
				if (!m_StartJob.IsCompletedSuccessfully)
				{
					Fail("Start manifest publication failed; native episode was not opened: " + m_StartJob.Exception!.GetBaseException().Message);
				}
				else if (TimeSync.IsAtSyncPoint)
				{
					ExportedArtifact = m_StartJob.Result;
					m_State = State.ValidationBeginQueued;
				}
			}
			else if (m_State == State.Exporting && m_ExportJob!.IsCompleted)
			{
				if (m_ExportJob.IsCompletedSuccessfully)
				{
					ExportedArtifact = m_ExportJob.Result;
					m_State = State.Complete;
					PublishFinishedEpisode();
					Status = "Episode exported. Native recording finalization has its own integrity checks. Prepare the next start when ready.";
				}
				else
				{
					Fail("Trajectory export failed: " + m_ExportJob.Exception!.GetBaseException().Message);
				}
			}
			if (m_State == State.Active && m_OwnsEpisode && !Observer.IsEpisodeOpen)
			{
				m_OwnsEpisode = false;
				Fail("Native recording ended outside this task. Prepare a new start before another episode.");
			}
		}

		protected override void OnLateUpdate(float deltaTime)
		{
			if (m_State == State.ValidationBeginQueued && TimeSync.IsAtSyncPoint)
			{
				if (!m_ExportThisEpisode || PrepareArtifact(publishStart: false))
				{
					OpenPreparedEpisode();
				}
			}
			if ((m_State != State.Active && m_State != State.EndQueued) || m_World == null)
			{
				return;
			}
			Vector3 position = m_World.GetPosition(m_TubeBody) + m_World.GetSceneTranslation();
			float restY = m_DeskY + m_TubeHeight * 0.5f;
			m_LiftObserved |= position.Y >= restY + m_MinimumLift;
			float dx = position.X - (m_Spawn.GoalLeft - m_Spawn.Width * 0.5f);
			float dz = position.Z - (m_Spawn.GoalBack - m_Spawn.Depth * 0.5f);
			Vector3 up = m_World.GetOrientation(m_TubeBody) * new Vector3(0f, 0f, 1f);
			float speed = m_World.GetVelocity(m_TubeBody).Length();
			float angularSpeed = m_World.GetAngularVelocity(m_TubeBody).Length();
			bool candidate = m_LiftObserved && dx * dx + dz * dz <= m_GoalTolerance * m_GoalTolerance
				&& MathF.Abs(position.Y - restY) <= m_HeightTolerance && up.Y >= 0.98f
				&& speed <= m_MaximumRestSpeed && angularSpeed <= m_MaximumAngularSpeed;
			m_StableElapsed = candidate ? m_StableElapsed + deltaTime : 0f;
			bool complete = m_StableElapsed >= m_StableSeconds;
			m_PlacementDiagnosticElapsed += deltaTime;
			if (m_PlacementDiagnosticElapsed >= 0.1f || complete != SimulatedPlacementComplete || m_State == State.EndQueued)
			{
				m_PlacementDiagnosticElapsed = 0f;
				PlacementErrorMm = MathF.Sqrt(dx * dx + dz * dz) * 1000f;
				PlacementHeightErrorMm = (position.Y - restY) * 1000f;
				PlacementTiltDegrees = MathF.Acos(Math.Clamp(up.Y, -1f, 1f)) * 180f / MathF.PI;
				PlacementSpeedMmPerSecond = speed * 1000f;
				PlacementAngularSpeedDegrees = angularSpeed * 180f / MathF.PI;
				PlacementCheck = complete ? "Simulated tube is upright and stable at the cross."
					: $"Simulation: cross error {PlacementErrorMm:F1} mm; height error {PlacementHeightErrorMm:+0.0;-0.0;0.0} mm; tilt {PlacementTiltDegrees:F1} deg; speed {PlacementSpeedMmPerSecond:F1} mm/s; spin {PlacementAngularSpeedDegrees:F1} deg/s; lift seen: {m_LiftObserved}.";
			}
			SimulatedPlacementComplete = complete;
			if (m_ExportThisEpisode && !m_Trajectory.Capture(m_World, TimeSync.CurrentSyncIndex, initial: false, out string error))
			{
				m_EndSuccess = false;
				m_CaptureValid = false;
				m_CaptureError = error;
				m_State = State.EndQueued;
				Status = "Capture invalid; finishing as unsuccessful: " + error;
			}
			MaximumFingerAsymmetryMm = m_Trajectory.MaximumObservedFingerAsymmetryMm;
			if (m_State == State.EndQueued && TimeSync.IsAtSyncPoint)
			{
				FinishEpisode();
			}
		}

		private bool Configure()
		{
			if (PhysicsWorld == null || Scene.FindEntityByHandle(PhysicsWorld.Handle) == null || (m_World = PhysicsWorld.GetComponent<MujocoSceneComponent>()) == null)
			{
				Fail("Assign the MuJoCo scene containing the free tube body, not a generated body entity.");
				return false;
			}
			m_TubeBody = m_World.GetBodyID(TubeBodyName);
			if (m_TubeBody == uint.MaxValue || m_TubeBody == 0 || m_World.GetBodyFreeJointIndex(m_TubeBody) == uint.MaxValue)
			{
				Fail("The named tube must exist and have a free joint in the assigned world.");
				return false;
			}
			if (ActiveRobotBase == null || Scene.FindEntityByHandle(ActiveRobotBase.Handle) == null)
			{
				Fail("Assign the active robot base before preparing a start.");
				return false;
			}
			float[] checks = { TubeHeight, TubeMassKg, SettleSeconds, MinimumLift, GoalTolerance, StableSeconds, HeightTolerance, MaximumRestSpeed, MaximumAngularSpeed };
			foreach (float value in checks)
			{
				if (!float.IsFinite(value) || value <= 0f)
				{
					Fail("Tube height and check tolerances/times must be finite positive values.");
					return false;
				}
			}
			if (!float.IsFinite(DeskSurfaceY))
			{
				Fail("Desk height must be finite.");
				return false;
			}
			m_Spawn.Width = DeskWidth; m_Spawn.Depth = DeskDepth; m_Spawn.Radius = TubeDiameter * 0.5f;
			m_Spawn.EdgeClearance = EdgeClearance; m_Spawn.LeftMin = LeftMin; m_Spawn.LeftMax = LeftMax;
			m_Spawn.BackMin = BackMin; m_Spawn.BackMax = BackMax; m_Spawn.GoalLeft = GoalLeft; m_Spawn.GoalBack = GoalBack;
			m_Spawn.GoalExclusionRadius = GoalExclusionRadius; m_Spawn.MaximumBaseDistance = MaximumBaseDistance; m_Spawn.BaseExclusionRadius = BaseExclusionRadius;
			m_Spawn.BaseCount = 0;
			AddBase(ActiveRobotBase); AddBase(OtherRobotBase1); AddBase(OtherRobotBase2); AddBase(OtherRobotBase3);
			m_WorldEntityId = (ulong)PhysicsWorld.Handle;
			m_PreparedSeed = Seed; m_PreparedOrdinal = EpisodeNumber; m_PreparedTaskIndex = TaskIndex;
			m_DeskY = DeskSurfaceY; m_TubeHeight = TubeHeight; m_TubeDiameter = TubeDiameter; m_TubeMass = TubeMassKg;
			m_SettleSeconds = SettleSeconds; m_MinimumLift = MinimumLift; m_GoalTolerance = GoalTolerance; m_StableSeconds = StableSeconds;
			m_HeightTolerance = HeightTolerance; m_MaximumRestSpeed = MaximumRestSpeed; m_MaximumAngularSpeed = MaximumAngularSpeed;
			return true;
		}

		private void AddBase(Entity? entity)
		{
			if (entity != null && Scene.FindEntityByHandle(entity.Handle) != null)
			{
				Vector3 position = entity.Transform.WorldTranslation;
				m_Spawn.Bases[m_Spawn.BaseCount++] = new TubeDeskPoint(position.X + DeskWidth * 0.5f, position.Z + DeskDepth * 0.5f);
			}
		}

		private void ResetTube()
		{
			if (Observer.IsEpisodeOpen || Scene.FindEntityByHandle((EntityHandle)m_WorldEntityId) == null)
			{
				Fail("Cannot reset while an episode is open or task setup is invalid.");
				return;
			}
			// Scope writes to this owner: generic body Transform setters currently broadcast to other MuJoCo instances.
			Quaternion axis = m_World!.GetAxisHzToMj();
			Vector3 baselineX = axis.Conjugate * Vector3.Right;
			float baselineHeading = MathF.Atan2(-baselineX.Z, baselineX.X);
			Quaternion worldYaw = new Quaternion(new Vector3(0f, m_Yaw - baselineHeading, 0f));
			// The tube's authored local up is native +Z. Preserve that basis while applying the requested world heading.
			Quaternion nativeRotation = axis * worldYaw * axis.Conjugate;
			m_World.WeldBody(m_TubeBody, axis * (m_Start - m_World.GetSceneTranslation()), nativeRotation, persistent: false);
			if (StartMarkerTemplate != null && Scene.FindEntityByHandle(StartMarkerTemplate.Handle) != null)
			{
				StaticMeshComponent? template = StartMarkerTemplate.GetComponent<StaticMeshComponent>();
				if (template != null)
				{
					if (m_StartMarker == null)
					{
						m_StartMarker = Scene.CreateEntity("Tube start mark (runtime visual)");
						m_StartMarker.CreateComponent<StaticMeshComponent>()!.Mesh = template.Mesh;
					}
					m_StartMarker.Transform.WorldScale = StartMarkerTemplate.Transform.WorldScale;
					m_StartMarker.Transform.WorldRotationQuat = StartMarkerTemplate.Transform.WorldRotationQuat;
					m_StartMarker.Transform.WorldTranslation = new Vector3(m_Start.X, m_DeskY + 0.0005f, m_Start.Z);
					m_StartMarker.GetComponent<StaticMeshComponent>()!.Visible = true;
				}
			}
			EpisodeNumber++;
			m_ResetSync = TimeSync.CurrentSyncIndex; m_Elapsed = 0f; m_StableElapsed = 0f;
			m_PlacementDiagnosticElapsed = 0f;
			PlacementErrorMm = PlacementHeightErrorMm = PlacementTiltDegrees = 0f;
			PlacementSpeedMmPerSecond = PlacementAngularSpeedDegrees = 0f;
			m_LiftObserved = false; SimulatedPlacementComplete = false;
			PlacementCheck = "Awaiting pick and place. Real object pose is not tracked by this simulation check.";
			m_State = State.Settling;
			Status = "Simulated tube settling before this episode can begin.";
		}

		private void QueueBegin(TubeWorkflow workflow)
		{
			if (Workflow != workflow || m_State != State.Ready)
			{
				Status = "Prepare a start and wait for it to settle, then use the begin button matching Workflow.";
				return;
			}
			if (RecordVirtualScene && Observer.IsRecording && !m_OwnsNativeSession)
			{
				Status = "Stop the externally started recording session first. This task must own a fresh session to map episode ordinals accurately.";
				return;
			}
			if (RecordVirtualScene && string.IsNullOrWhiteSpace(NativeSessionLabel))
			{
				Status = "Enter the exact configured native Recorder session label before starting virtual recording; output paths are not guessed.";
				return;
			}
			m_Workflow = workflow; m_RecordEpisode = RecordVirtualScene; m_ExportThisEpisode = ExportTrajectory;
			// Rejected button clicks still cancel automatic intent, but they do not
			// transfer ownership of an already accepted Begin to another operation.
			m_BeginControlRevision = ControlRevision;
			m_State = State.BeginQueued;
			Status = "Start confirmed; beginning at the next sync point.";
		}

		private void BeginEpisode()
		{
			m_EpisodeId = Guid.NewGuid().ToString("N");
			if (!EpisodeFollowers.Begin(m_WorldEntityId, m_EpisodeId, (uint)m_PreparedOrdinal))
			{
				Fail("The previous physical follower has not completed. This episode was not opened.");
				return;
			}
			if (m_StartMarker != null)
			{
				m_StartMarker.GetComponent<StaticMeshComponent>()!.Visible = false;
			}
			if (m_ExportThisEpisode)
			{
				PrepareArtifact(publishStart: true);
				return;
			}
			m_State = State.ValidationBeginQueued;
		}

		private void OpenPreparedEpisode()
		{
			if (m_RecordEpisode)
			{
				Observer.RegisterTask(m_PreparedTaskIndex, "Pick the upright wood-filler tube and place it at the desk cross. Task session: " + m_SessionId);
				Observer.SetCurrentTaskIndex(m_PreparedTaskIndex);
				bool startingSession = !Observer.IsRecording;
				if (!startingSession)
				{
					Observer.BeginEpisode();
				}
				else
				{
					m_OwnsNativeSession = false;
					Observer.StartRecording();
				}
				if (!Observer.IsEpisodeOpen)
				{
					Fail("Native recorder did not open an episode. Check its output/stream configuration; nothing is recording.");
					return;
				}
				if (startingSession)
				{
					m_OwnsNativeSession = true;
					m_NativeOrdinal = 0;
				}
				m_OwnsEpisode = true;
				m_NativeOrdinal++;
			}
			m_State = State.Active;
			Status = m_Workflow == TubeWorkflow.RealDemonstration
				? "Pick and place the REAL tube, then release it upright at the cross and finish. This task does not record real sensors."
				: "Demonstrate simulated pick and place, release the tube upright at the cross, then finish.";
		}

		private bool PrepareArtifact(bool publishStart)
		{
			if (m_Workflow != TubeWorkflow.SimulationOnly)
			{
				Fail("Trajectory export currently supports simulation only. The plugin's physical feedback is Edit-only.");
				return false;
			}
			if (!m_Trajectory.Configure(m_World!, m_Qualification, out string error) || !m_Trajectory.Begin(m_World!, TimeSync.CurrentSyncIndex, out error))
			{
				Fail(error);
				return false;
			}
			uint nq = m_World!.ModelNq;
			if (nq == 0 || nq > 512)
			{
				Fail("Unexpected task model qpos size. Export requires the verified Piper/tube model.");
				return false;
			}
			float[] initialQpos = new float[nq];
			if (m_World.GetFullQpos(initialQpos) != initialQpos.Length)
			{
				Fail("Could not capture the complete initial simulation qpos state.");
				return false;
			}
			foreach (float value in initialQpos)
			{
				if (!float.IsFinite(value))
				{
					Fail("Initial simulation state is not finite.");
					return false;
				}
			}
			Vector3 position = m_World.GetPosition(m_TubeBody) + m_World.GetSceneTranslation();
			Quaternion rotation = m_World.GetOrientation(m_TubeBody);
			if ((position - m_Start).Length() > m_HeightTolerance * 2f || (rotation * new Vector3(0f, 0f, 1f)).Y < 0.98f
				|| m_World.GetVelocity(m_TubeBody).Length() > m_MaximumRestSpeed || m_World.GetAngularVelocity(m_TubeBody).Length() > m_MaximumAngularSpeed)
			{
				Fail("The tube moved away from its prepared start before capture. Prepare again; no episode was opened.");
				return false;
			}
			Vector3 bodyX = rotation * Vector3.Right;
			float yaw = MathF.Atan2(-bodyX.Z, bodyX.X);
			m_Artifact = new TubeEpisodeExport
			{
				ArtifactId = m_EpisodeId, SessionId = m_SessionId,
				CreatedUtc = DateTimeOffset.UtcNow.ToString("O"), Seed = m_PreparedSeed, EpisodeOrdinal = m_PreparedOrdinal,
				Robot = new { model = "agilex.piper", jointConvention = "piper-j1-j6-degrees-v1", jointNames = new[] { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" }, gripperConvention = "pika-opening-mm-v1", gripperMaximumOpeningMm = m_Trajectory.MaximumOpeningMm, gripperReferenceGapMm = m_Trajectory.GripperReferenceGapMm, gripperMapping = "pika-opposed-slides-open-at-zero-v1", commandConvention = "piper-position-actuator-targets-degrees-pika-mm-v1", actionSource = "measured qpos observations plus verified position-actuator ctrl targets", positionActuatorProvenance = new { sourceSha256 = m_Qualification.SourceSha256, validation = TubeActuatorQualification.ValidationDescription, runtimeChecks = TubeActuatorQualification.RuntimeChecksDescription, runtimeTuning = TubeActuatorQualification.RuntimeTuningDescription } },
				Desk = new { widthMm = m_Spawn.Width * 1000f, depthMm = m_Spawn.Depth * 1000f, surfaceYMetres = m_DeskY, frame = "engine-world-y-up-centred-xz-back-negative-z" },
				TubeDimensions = new { diameterMm = m_TubeDiameter * 1000f, heightMm = m_TubeHeight * 1000f, massKg = m_TubeMass, massSource = "estimated" },
				TubeStart = new { leftMm = (position.X + m_Spawn.Width * 0.5f) * 1000f, backMm = (position.Z + m_Spawn.Depth * 0.5f) * 1000f, yawDeg = yaw * 180f / MathF.PI, worldPositionMetres = new { x = position.X, y = position.Y, z = position.Z }, worldQuaternionXyzw = new[] { rotation.X, rotation.Y, rotation.Z, rotation.W }, source = "settled simulation body immediately before episode begin" },
				Goal = new { leftMm = m_Spawn.GoalLeft * 1000f, backMm = m_Spawn.GoalBack * 1000f, placementToleranceMm = m_GoalTolerance * 1000f },
				NativeRecording = new { requested = m_RecordEpisode, sessionLabel = NativeSessionLabel, sessionLabelSource = "operator supplied; not discovered from native Recorder", episodeOrdinal = m_RecordEpisode ? (Observer.IsRecording ? m_NativeOrdinal : 0) : (int?)null, nativeOrdinalSource = "task_owned_session_counter_not_engine_reported", taskIndex = m_PreparedTaskIndex, taskSessionId = m_SessionId },
				InitialSimulationState = new { mujocoQpos = initialQpos, qposConvention = "native MuJoCo model order: hinges radians, slides/free translations metres, free quaternions wxyz; not a hardware command", worldEntityId = m_WorldEntityId.ToString(), syncIndex = TimeSync.CurrentSyncIndex },
				Samples = publishStart ? Array.Empty<TubeReplaySample>() : m_Trajectory.Snapshot()
			};
			if (publishStart)
			{
				if (string.IsNullOrWhiteSpace(ExportDirectory) || !Path.IsPathFullyQualified(ExportDirectory))
				{
					Fail("Choose an absolute export directory before starting an episode.");
					return false;
				}
				m_ExportDirectory = Path.Combine(ExportDirectory, m_SessionId);
				m_StartJob = WoodFillerTrajectory.PublishStartAsync(m_ExportDirectory, m_Artifact);
				m_State = State.PublishingStart;
				Status = "Saving the start manifest before opening this episode.";
			}
			m_CaptureValid = true;
			m_CaptureError = "";
			return true;
		}

		private void FinishEpisode()
		{
			if (m_EndSuccess && m_Workflow == TubeWorkflow.SimulationOnly && !SimulatedPlacementComplete)
			{
				m_State = State.Active;
				Status = "The tube moved before finishing. Release it upright at the cross and wait until it is still.";
				return;
			}
			if (m_ExportThisEpisode && (!m_CaptureValid || m_Trajectory.Count < 2))
			{
				m_EndSuccess = false;
			}
			if (m_OwnsEpisode)
			{
				Observer.EndCurrentEpisode(m_EndSuccess);
				m_OwnsEpisode = false;
			}
			if (m_ExportThisEpisode)
			{
				m_Artifact!.Samples = m_Trajectory.Snapshot();
				m_Artifact.Outcome = !m_CaptureValid ? "invalid-capture: " + m_CaptureError : m_Trajectory.Count < 2 ? "invalid-capture: fewer than two samples" : m_EndSuccess ? "success" : "unsuccessful";
				m_ExportJob = WoodFillerTrajectory.PublishFinishedAsync(m_ExportDirectory, m_Artifact);
				m_State = State.Exporting;
				Status = "Publishing the trajectory and start manifest. No robot commands are being sent.";
			}
			else
			{
				m_State = State.Complete;
				PublishFinishedEpisode();
				Status = m_EndSuccess ? "Episode labelled successful. Prepare the next start when ready." : "Episode labelled unsuccessful. Prepare the next start when ready.";
			}
		}

		private void PublishFinishedEpisode()
		{
			if (m_EpisodeId.Length != 0)
			{
				EpisodeFollowers.Finish(m_WorldEntityId, m_EpisodeId, m_EndSuccess && (!m_ExportThisEpisode || m_CaptureValid),
					m_ExportThisEpisode ? ExportedArtifact : "");
			}
		}

		private void QueueEnd(bool success)
		{
			if (m_State != State.Active)
			{
				Status = "No task episode is active.";
				return;
			}
			if (success && m_Workflow == TubeWorkflow.SimulationOnly && !SimulatedPlacementComplete)
			{
				Status = "Lift the simulated tube, then place it upright and still at the cross before confirming success.";
				return;
			}
			m_EndSuccess = success; m_State = State.EndQueued;
			Status = "Finishing at the next sync point.";
		}

		private void Fail(string message)
		{
			if (m_EpisodeId.Length != 0) EpisodeFollowers.Finish(m_WorldEntityId, m_EpisodeId, false);
			if (m_OwnsEpisode)
			{
				Observer.EndCurrentEpisode(false);
				m_OwnsEpisode = false;
			}
			m_State = State.Error;
			Status = message;
		}
	}
}
