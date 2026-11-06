using System;
using Hazel;
using CanTake;

namespace LuckyExporter
{
	/// <summary>
	/// A point for the robot's fingertips (the grasp site between them) to move to: this entity's own
	/// origin and rotation. The entity's +Y is the way the fingers point and its X the way the jaws
	/// open. The director eases the hand to this entity's world position over Duration (after Wait
	/// Start), then waits Wait End before the next task.
	///
	/// The LD_Goal and LD_Target prefabs carry this script. A marker without it moves with the default
	/// settings (RunDefault). LuckyEngine's prefab editor can't load project scripts and may crash on
	/// prefabs that have one, so adjust the instances in the scene.
	///
	/// Grabber Action runs on its own timer: Grabber Delay seconds after the move starts, the gripper
	/// starts opening or closing and takes Grabber Time to get there, while the arm keeps moving,
	/// after it arrives, or as the director moves on.
	///
	/// Immediate has the director drive the move itself (RobotDirector.StartStreamedMove) rather than
	/// through Run, as the graph's eased move always comes to a stop at both ends.
	/// </summary>
	public class LD_Waypoint : Entity
	{
		internal const int GrabNone = 0;
		internal const int GrabOpen = 1;
		internal const int GrabClose = 2;
		internal const int GrabSet = 3;

		private const int FixedOrientation = 0;
		private const int FromWaypoint = 1;
		private const float DegPerRad = 180.0f / MathF.PI;

		[Dropdown("Left Arm", "Right Arm")]
		[Tooltip("Which hand moves here. The other keeps its last target. Ignored on a single-arm robot (Panda).")]
		public int Arm = 0;

		[Units("s")]
		[Slider(0.1f, 10.0f)]
		[Tooltip("Time to ease from the current pose to this waypoint.")]
		public float Duration = 0.56f;

		[Tooltip("Carry straight on from the previous waypoint without stopping there: the hand travels in a straight line to this one and keeps its speed through the previous waypoint. Needs no Wait End on the previous waypoint, no Wait Start here, the same Arm, and no other task between them; otherwise it sets off smoothly from rest, still in a straight line.")]
		public bool Immediate = false;

		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait before this task starts.")]
		public float WaitStart = 0.106f;

		[DisplayName("Wait End")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait after arriving before the next task starts.")]
		public float Hold = 0f; // shown as Wait End; the field name keeps saved values

		[ToggleButton("Solve Orientation", "Position Only")]
		[Tooltip("Also turn the hand, to this waypoint's rotation or a fixed Orientation. Position Only leaves the wrist wherever the position solve puts it.")]
		public bool SolveOrientation = true;

		[EditCondition("SolveOrientation")]
		[Dropdown("Fixed", "From Waypoint")]
		[Tooltip("From Waypoint matches the claw to this entity's rotation, so rotating the marker turns the hand. Fixed holds Orientation.")]
		public int OrientationSource = FromWaypoint;

		[Units("deg")]
		[EditCondition("SolveOrientation")]
		[Tooltip("Fixed: the hand's orientation in world XYZ degrees. From Waypoint: an extra rotation in this entity's own frame, on top of its rotation (the fingers point along its +Y and the jaws open along its X).")]
		public Vector3 Orientation = Vector3.Zero;

		[DisplayName("Grabber Action")]
		[Dropdown("None", "Open", "Close", "Set")]
		[Tooltip("Opens or closes the gripper of this task's arm (Set: to Grabber Value) while the arm moves: it starts Grabber Delay seconds after the move starts and takes Grabber Time. It runs on its own timer and never holds up the tasks.")]
		public int GrabberAction = GrabNone;

		[DisplayName("Grabber Value")]
		[Slider(0.0f, 1.0f)]
		[Tooltip("For Set: how far open the gripper goes, from 0 (closed) to 1 (fully open), on any robot's gripper range.")]
		public float GrabberValue = 0f;

		[DisplayName("Grabber Delay")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Seconds after the move starts (after Wait Start) that the gripper starts opening or closing. 0 starts with the move.")]
		public float GrabberDelay = 0.0f;

		[DisplayName("Grabber Time")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("How long the gripper takes to open or close. Set it to Duration (with Grabber Delay 0) to close across the whole move; 0 snaps.")]
		public float GrabberTime = 1.0f;

		/// <summary>Starts the move and returns its Duration; the director waits that, then Hold. Any grab is scheduled separately.
		/// hand, when given, is the world rotation for the grasp site instead of this waypoint's own (also with Position Only).</summary>
		public float Run(RobotControllerComponent controller, bool singleArm = false, Quaternion? hand = null)
			=> Move(this, controller, singleArm, Arm, Duration, SolveOrientation, OrientationSource, Orientation, hand);

		/// <summary>Moves the hand to a marker entity without an LD_Waypoint, with the default settings above.</summary>
		public static float RunDefault(Entity marker, RobotControllerComponent controller, bool singleArm, Quaternion? hand = null)
			=> Move(marker, controller, singleArm, 0, DefaultDuration, true, FromWaypoint, Vector3.Zero, hand);

		/// <summary>The grasp site's world rotation this waypoint asks for with Solve Orientation.</summary>
		internal Quaternion HandRotation() => HandQuaternion(this, OrientationSource, Orientation);

		/// <summary>The grasp site's world rotation a marker without an LD_Waypoint asks for.</summary>
		internal static Quaternion DefaultHandRotation(Entity marker) => HandQuaternion(marker, FromWaypoint, Vector3.Zero);

		internal const float DefaultDuration = 2.0f;

		private static float Move(Entity target, RobotControllerComponent controller, bool singleArm, int arm, float duration,
			bool solveOrientation, int orientationSource, Vector3 orientation, Quaternion? hand = null)
		{
			solveOrientation |= hand.HasValue;
			controller.SetInputBool(RobotInput.LeftArm_Solve_Orientation.ForArm(arm, singleArm), solveOrientation);
			if (solveOrientation)
				controller.SetInputVector3(RobotInput.LeftArm_Target_Orientation.ForArm(arm, singleArm),
					hand.HasValue ? hand.Value.Normalized().EulerAngles * DegPerRad : HandOrientation(target, orientationSource, orientation));
			controller.SetInputVector3(RobotInput.LeftArm_Target_Position.ForArm(arm, singleArm), target.Transform.WorldTranslation);
			controller.SetInputFloat(RobotInput.Duration.ID(), duration);
			controller.SetInputTrigger(RobotInput.Start.ID());
			return duration;
		}

		// The grasp site's frame (Z along the fingers, Y the jaw opening, X = Y x Z) relative to this
		// entity: fingers along +Y, jaws along X, so hand X is the entity's Z
		private static readonly Vector3 MarkerToHand = new Vector3(-90.0f, -90.0f, 0.0f);

		/// <summary>The grasp site's rotation relative to a marker (LD_Waypoint, or a curve's waypoint arrow): the fingers along its +Y, the jaws along its X.</summary>
		internal static Quaternion MarkerToHandRotation => PikaHand
			? (new Quaternion(MarkerToHand / DegPerRad) * Quaternion.AngleAxis(-90.0f, new Vector3(0.0f, 1.0f, 0.0f))).Normalized()
			: new Quaternion(MarkerToHand / DegPerRad);

		/// <summary>The Pika gripper's grasp site has X along the fingers and Y the jaw opening, unlike the Z-along-the-fingers frame above. Set from the robot by RobotDirector and CurveFollower, so a project's robots must share one hand type.</summary>
		internal static bool PikaHand;

		/// <summary>The axis the fingers point along, in the grasp site's frame.</summary>
		internal static Vector3 FingerAxis => PikaHand ? new Vector3(1.0f, 0.0f, 0.0f) : new Vector3(0.0f, 0.0f, 1.0f);

		// World XYZ degrees for the hand
		private static Vector3 HandOrientation(Entity target, int orientationSource, Vector3 orientation)
		{
			if (orientationSource == FixedOrientation)
				return orientation;
			return HandQuaternion(target, orientationSource, orientation).EulerAngles * DegPerRad;
		}

		private static Quaternion HandQuaternion(Entity target, int orientationSource, Vector3 orientation)
		{
			if (orientationSource == FixedOrientation)
				return new Quaternion(orientation / DegPerRad);
			Quaternion hand = MarkerToHandRotation;
			Quaternion offset = new Quaternion(orientation / DegPerRad);
			return (target.Transform.WorldRotationQuat * offset * hand).Normalized();
		}
	}
}
