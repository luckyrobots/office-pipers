using System;
using Hazel;

namespace CanTake
{
	/// <summary>
	/// Moves an IK target of the OpenArm pedestal (either arm) or a single-arm robot such as Panda along a CurvePath. On Play the hand eases
	/// onto the start of the curve, then the target travels along it at a constant speed; the other
	/// arm is held where it started. Every field can be changed while playing: switching Arm holds the
	/// old hand where it is and eases the new one onto the curve. The motion itself is
	/// <see cref="CurveMotion"/>, which RobotDirector also uses for curve tasks.
	/// </summary>
	public class CurveFollower : Entity
	{
		[Tooltip("Off: the follower sends nothing and the hand stays where it is. Switching on while playing eases the hand back onto the start of the curve.")]
		public bool Enabled = true;

		[Tooltip("The CurvePath entity whose curve the hand follows.")]
		public Entity? Curve;

		[Tooltip("The robot root entity, with the RobotController and MujocoScene components.")]
		public Entity? Robot;

		[Dropdown("Left Arm", "Right Arm")]
		[Tooltip("Which hand follows the curve. The other arm holds its starting pose.")]
		public int Arm = 0;

		[Units("m/s")]
		[Slider(0.01f, 2.0f)]
		[Tooltip("Speed of the target along the curve. The arm may lag behind this at high speed.")]
		public float Speed = 0.5f;

		[ToggleButton("Ping-Pong", "Loop")]
		[Tooltip("Ping-Pong runs back and forth along an open curve; Loop jumps back to the start (or wraps, on a closed curve).")]
		public bool PingPong = true;

		[ToggleButton("Reverse", "Forward")]
		[Tooltip("Reverse starts from the last waypoint and travels toward the first. Switching while playing turns the hand around where it is.")]
		public bool Reverse = false;

		[Units("s")]
		[Slider(0.2f, 5.0f)]
		[Tooltip("Time to ease the hand from its starting pose onto the start of the curve.")]
		public float ApproachTime = 0.4f;

		[ToggleButton("Solve Orientation", "Position Only")]
		[Tooltip("Also control the hand's orientation, from the waypoints or a fixed Hand Orientation. Position Only leaves the wrist wherever the position solve puts it.")]
		public bool SolveOrientation = true;

		[EditCondition("SolveOrientation")]
		[Dropdown("Fixed", "From Waypoints")]
		[Tooltip("From Waypoints turns the hand with the waypoints: at each waypoint the fingers point the way its arrow points (+Y) and the jaws open along its X, as with an LD_Waypoint, blending to the next one along the way. Fixed holds Hand Orientation the whole time.")]
		public int OrientationSource = CurveMotion.FromWaypoints;

		[Units("deg")]
		[EditCondition("SolveOrientation")]
		[Tooltip("Fixed: the hand's orientation in world XYZ degrees. From Waypoints: an extra rotation in each waypoint's own frame, on top of its arrow (the fingers point along its +Y and the jaws open along its X).")]
		public Vector3 HandOrientation = Vector3.Zero;

		[ToggleButton("High Accuracy", "Normal")]
		[Tooltip("Spend more solver iterations per update, so the hand lags less behind a fast target.")]
		public bool HighAccuracy = false;

		private CurvePath? m_Path;
		private CurveMotion? m_Motion;
		private bool m_WasEnabled;

		// The editor's prefab preview creates scripts on copies outside the running scene, where any
		// engine call crashes the editor. So setup waits for the first update (Play only) and a copy
		// the scene doesn't hold does nothing at all
		private bool m_Started;
		private bool m_Live;

		private bool StartOnce()
		{
			if (!m_Started)
			{
				m_Started = true;
				m_Live = this; // Entity converts to bool through Scene.IsEntityValid
				if (m_Live)
					Setup();
			}
			return m_Live;
		}

		private void Setup()
		{
			RobotControllerComponent? controller = Robot?.GetComponent<RobotControllerComponent>();
			m_Path = Curve?.As<CurvePath>();
			if (controller is null || m_Path is null)
			{
				Log.Warn("[CurveFollower] Set Curve to a CurvePath entity and Robot to the robot root.");
				return;
			}

			// Single-arm robots (Panda) have a "grasp_site" and unprefixed inputs (Target_Position, ...)
			bool singleArm = RobotInputs.IsSingleArm(Robot!);
			LuckyExporter.LD_Waypoint.PikaHand = RobotInputs.HasPikaHand(Robot!);
			m_Motion = new CurveMotion(Robot!, controller, singleArm);
			CopySettings();
			if (singleArm)
				Log.Info("[CurveFollower] Single-arm robot detected; Arm setting is ignored.");
			else
				m_Motion.HoldArm(1 - Arm);
			m_WasEnabled = Enabled;
			if (Enabled)
				m_Motion.Begin(m_Path);
		}

		protected override void OnUpdate(float deltaTime)
		{
			if (!StartOnce())
				return;
			if (m_Motion is null || m_Path is null)
				return;

			// Copied every update, so inspector changes apply while playing
			CopySettings();
			if (Enabled != m_WasEnabled)
			{
				m_WasEnabled = Enabled;
				if (Enabled)
					m_Motion.Begin(m_Path);
				else
					m_Motion.HoldCurrentArm(); // stop where it is rather than finishing the last target
			}
			if (Enabled)
				m_Motion.Update(deltaTime);
		}

		private void CopySettings()
		{
			m_Motion!.Arm = Arm;
			m_Motion.Speed = Speed;
			m_Motion.PingPong = PingPong;
			m_Motion.Reverse = Reverse;
			m_Motion.ApproachTime = ApproachTime;
			m_Motion.SolveOrientation = SolveOrientation;
			m_Motion.OrientationSource = OrientationSource;
			m_Motion.HandOrientation = HandOrientation;
			m_Motion.HighAccuracy = HighAccuracy;
		}
	}

	/// <summary>
	/// Moves one robot hand along a CurvePath: Begin eases the hand onto the start of the curve,
	/// then each Update moves the target along it at Speed. Settings can be changed between
	/// updates. With Once set it stops at the end of one pass and sets Finished; otherwise it runs
	/// until told to stop. Used by CurveFollower, and by RobotDirector for curve tasks.
	/// </summary>
	public sealed class CurveMotion
	{
		public const int FixedOrientation = 0;
		public const int FromWaypoints = 1;
		private const float DegPerRad = 180.0f / MathF.PI;
		private const float MaxCatchUpSpeed = 0.25f; // m/s

		public int Arm = 0;
		public float Speed = 0.5f;
		public bool PingPong = true;
		public bool Reverse = false;
		public float ApproachTime = 0.4f;
		public bool SolveOrientation = true;
		public int OrientationSource = FromWaypoints;
		public Vector3 HandOrientation = Vector3.Zero;
		public bool HighAccuracy = false;
		public bool TrackingMode = true;

		/// <summary>One pass: stop at the end of the curve (back at the start, on a closed curve) and set Finished. Ping-Pong and Loop are ignored.</summary>
		public bool Once = false;

		/// <summary>Set when a Once pass reaches the end; the target then stays there.</summary>
		public bool Finished { get; private set; }

		private readonly Entity m_Robot;
		private readonly RobotControllerComponent m_Controller;
		private readonly bool m_SingleArm;
		private CurvePath? m_Path;
		private int m_Arm = -1;
		private float m_Time;
		private float m_Distance;
		private int m_Direction = 1;
		private Vector3[] m_LastPath = Array.Empty<Vector3>();
		private Vector3 m_Target;
		private Quaternion[] m_Rotations = Array.Empty<Quaternion>(); // world rotation at each point of the last WorldPath
		private bool m_HasTarget;

		public CurveMotion(Entity robot, RobotControllerComponent controller, bool singleArm)
		{
			m_Robot = robot;
			m_Controller = controller;
			m_SingleArm = singleArm;
		}

		/// <summary>Starts on a curve: eases the hand from wherever it is onto the curve's start.</summary>
		public void Begin(CurvePath path)
		{
			m_Path = path;
			Finished = false;
			BeginApproach();
		}

		/// <summary>Keeps an arm still at its hand's current position, so a Start pulse doesn't chase (0, 0, 0).</summary>
		public void HoldArm(int arm) => RobotInputs.HoldArm(m_Robot, m_Controller, arm, m_SingleArm);

		/// <summary>Stops the arm that's following where it is.</summary>
		public void HoldCurrentArm()
		{
			if (m_Arm >= 0)
				HoldArm(m_Arm);
		}

		// The graph input for this arm: LeftArm_X / RightArm_X on OpenArm, plain X on a single-arm robot
		private Identifier In(RobotInput leftArmInput, int arm) => leftArmInput.ForArm(arm, m_SingleArm);

		public void Update(float deltaTime)
		{
			if (m_Path is null || Finished)
				return;

			if (!m_SingleArm && Arm != m_Arm)
			{
				HoldArm(m_Arm);
				BeginApproach();
			}
			ApplySolverOptions();

			Vector3[] path = WorldPath();
			if (path.Length == 0)
				return;

			// While easing on, keep the target on the curve's start so moving the curve is followed
			m_Time += deltaTime;
			if (m_Time < ApproachTime)
			{
				m_LastPath = path;
				SetTarget(path[0], RotationAt(0));
				return;
			}

			float total = 0.0f;
			for (int i = 1; i < path.Length; i++)
				total += (path[i] - path[i - 1]).Length();
			if (total < 1e-4f)
			{
				SetTarget(path[0], RotationAt(0));
				Finished = Once;
				return;
			}

			// A waypoint was added, removed or moved: the same distance now lands somewhere else on
			// the curve, so carry on from the nearest point to where the hand was heading
			if (m_HasTarget && PathChanged(path))
				m_Distance = ClosestDistance(path, m_Target);
			m_LastPath = path;

			m_Distance += Speed * deltaTime * m_Direction;
			if (Once)
			{
				if (m_Distance >= total)
				{
					m_Distance = total;
					Finished = true;
				}
			}
			else if (PingPong && !m_Path.Closed)
			{
				m_Distance = Math.Clamp(m_Distance, -total, 2.0f * total); // the curve can shrink under the hand
				if (m_Distance > total) { m_Distance = 2.0f * total - m_Distance; m_Direction = -1; }
				if (m_Distance < 0.0f) { m_Distance = -m_Distance; m_Direction = 1; }
			}
			else
			{
				m_Distance = ((m_Distance % total) + total) % total;
				m_Direction = 1;
			}

			// Ease toward the point instead of jumping, in case the curve moved away from the hand
			Vector3 point = PointAt(path, m_Distance, out int index, out float fraction);
			float maxStep = MathF.Max(Speed * 2.0f, MaxCatchUpSpeed) * deltaTime;
			Vector3 step = point - m_Target;
			SetTarget(step.Length() > maxStep ? m_Target + step.Normalized() * maxStep : point, RotationAt(index, fraction));
		}

		private bool PathChanged(Vector3[] path)
		{
			if (path.Length != m_LastPath.Length)
				return true;
			for (int i = 0; i < path.Length; i++)
			{
				if (!Vector3.EpsilonEquals(path[i], m_LastPath[i]))
					return true;
			}
			return false;
		}

		private static float ClosestDistance(Vector3[] path, Vector3 point)
		{
			float best = float.MaxValue, bestDistance = 0.0f, along = 0.0f;
			for (int i = 1; i < path.Length; i++)
			{
				Vector3 a = path[i - 1], ab = path[i] - a;
				float span = ab.Length();
				float t = span > 0.0f ? Math.Clamp(Vector3.Dot(point - a, ab) / (span * span), 0.0f, 1.0f) : 0.0f;
				float d = (a + ab * t - point).Length();
				if (d < best)
				{
					best = d;
					bestDistance = along + span * t;
				}
				along += span;
			}
			return bestDistance;
		}

		// Ease the selected hand from wherever it is onto the start of the curve
		private void BeginApproach()
		{
			m_Arm = Arm;
			m_Time = 0.0f;
			m_Distance = 0.0f;
			m_Direction = 1;

			m_Controller.SetInputBool(In(RobotInput.LeftArm_Tracking_Mode, m_Arm), TrackingMode);
			ApplySolverOptions();
			Vector3[] path = WorldPath();
			if (path.Length > 0)
				SetTarget(path[0], RotationAt(0));
			m_Controller.SetInputFloat(RobotInput.Duration.ID(), ApproachTime);
			m_Controller.SetInputTrigger(RobotInput.Start.ID());
		}

		// Cheap enough to send every update, so setting changes apply while playing
		private void ApplySolverOptions()
		{
			m_Controller.SetInputBool(In(RobotInput.LeftArm_Solve_Orientation, m_Arm), SolveOrientation);
			if (OrientationSource == FixedOrientation)
				m_Controller.SetInputVector3(In(RobotInput.LeftArm_Target_Orientation, m_Arm), HandOrientation);
			m_Controller.SetInputBool(In(RobotInput.LeftArm_High_Accuracy, m_Arm), HighAccuracy);
		}

		// rotation is the curve's world rotation at the point, used when orientation comes from the waypoints
		private void SetTarget(Vector3 position, Quaternion rotation)
		{
			m_Target = position;
			m_HasTarget = true;
			m_Controller.SetInputVector3(In(RobotInput.LeftArm_Target_Position, m_Arm), position);
			if (SolveOrientation && OrientationSource == FromWaypoints)
			{
				// Hand Orientation is an offset in the waypoint's own frame. As with LD_Waypoint, each
				// waypoint arrow's +Y is the way the fingers point and its X the way the jaws open
				Quaternion offset = new Quaternion(HandOrientation / DegPerRad);
				Vector3 euler = (rotation * offset * LuckyExporter.LD_Waypoint.MarkerToHandRotation).Normalized().EulerAngles * DegPerRad;
				m_Controller.SetInputVector3(In(RobotInput.LeftArm_Target_Orientation, m_Arm), euler);
			}
		}

		private Quaternion RotationAt(int index, float fraction = 0.0f)
		{
			if (m_Rotations.Length == 0)
				return Quaternion.Identity;
			if (index >= m_Rotations.Length - 1)
				return m_Rotations[m_Rotations.Length - 1];
			return Quaternion.Slerp(m_Rotations[index], m_Rotations[index + 1], fraction);
		}

		// The sampled curve in world space, in travel order; also refreshes m_Rotations to match
		private Vector3[] WorldPath()
		{
			Vector3[] local = m_Path!.SampleLocal();
			TransformComponent t = m_Path.Transform;
			Vector3 position = t.WorldTranslation;
			Quaternion rotation = t.WorldRotationQuat;
			Vector3 scale = t.WorldScale;
			var world = new Vector3[local.Length];
			for (int i = 0; i < local.Length; i++)
				world[i] = position + rotation * new Vector3(local[i].X * scale.X, local[i].Y * scale.Y, local[i].Z * scale.Z);

			Quaternion[] localRotations = m_Path.SampleRotationsLocal();
			m_Rotations = new Quaternion[localRotations.Length];
			for (int i = 0; i < localRotations.Length; i++)
				m_Rotations[i] = rotation * localRotations[i];

			// Reversed, the curve starts at its last waypoint; toggling it while playing reads as a
			// path change, so the hand carries on from where it is
			if (Reverse)
			{
				Array.Reverse(world);
				Array.Reverse(m_Rotations);
			}
			return world;
		}

		// The point at a distance along the path, and where it falls: between points index and index + 1, at fraction
		private static Vector3 PointAt(Vector3[] path, float distance, out int index, out float fraction)
		{
			for (int i = 1; i < path.Length; i++)
			{
				float span = (path[i] - path[i - 1]).Length();
				if (distance <= span && span > 0.0f)
				{
					index = i - 1;
					fraction = distance / span;
					return path[i - 1] + (path[i] - path[i - 1]) * fraction;
				}
				distance -= span;
			}
			index = path.Length - 1;
			fraction = 0.0f;
			return path[path.Length - 1];
		}
	}
}
