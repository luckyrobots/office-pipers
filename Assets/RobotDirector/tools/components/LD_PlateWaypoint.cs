using System;
using Hazel;

namespace LuckyExporter
{
	/// <summary>
	/// One step for LD_PlateDirector: moves Robot's fingertips (the grasp site between them) to this entity's
	/// position and rotation, and can work its gripper. With Use Elbow Pole on, the direction from this entity
	/// to its Elbow Pole child, the sphere, is the way the elbow is pulled: how far away the sphere is makes no
	/// difference. As with LD_Waypoint, this entity's +Y is the way the
	/// fingers point and its X the way the jaws open. The director runs these in name order.
	/// </summary>
	public class LD_PlateWaypoint : Entity
	{
		internal const string ElbowPoleName = "Elbow Pole";
		private const float MinPoleOffset = 0.001f; // a sphere on its waypoint gives no direction

		internal const int GripperNone = 0;
		internal const int CloseOnStart = 1;
		internal const int CloseOnEnd = 2;
		internal const int OpenOnStart = 3;
		internal const int OpenOnEnd = 4;

		internal const int RestNone = 0;
		internal const int RestOnStart = 1;
		internal const int RestOnEnd = 2;

		[Tooltip("The Piper that moves here. It must be a child of the director's Robots entity.")]
		public Entity? Robot;

		[Units("s")]
		[Slider(0.1f, 20.0f)]
		[Tooltip("Time to ease from the robot's current target to this waypoint.")]
		public float Duration = 1.2f;

		[DisplayName("Solve Rotation")]
		[Tooltip("Off: the hand goes to this position at whatever rotation suits the arm, which frees it to reach further and lets the Elbow Pole move the elbow much more. Leave it on where the hand's rotation matters, such as gripping.")]
		public bool SolveRotation = false;

		[DisplayName("Use Elbow Pole")]
		[Tooltip("Pull the elbow in the direction from this waypoint to the Elbow Pole sphere under it. Only the direction counts, not the distance. The hand still goes to this waypoint.")]
		public bool UseElbowPole = false;

		[DisplayName("Pole Weight")]
		[Slider(0.0f, 1.0f)]
		[Tooltip("How strongly the sphere pulls the elbow. With Solve Rotation on, keep it small: the arm has no spare joints, so the pull also moves the hand off the waypoint, by about 6 mm at 0.02 and 30 mm at 0.1.")]
		public float PoleWeight = 0.466f;

		[Tooltip("Don't stop here: the hand passes through at speed and carries on into the next waypoint, for one smooth motion. Needs the next waypoint to be the same robot's, with no wait, gripper move at the end or rest between them.")]
		public bool Continuous = true;

		[DisplayName("Overlap From Last")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("0: wait for every earlier waypoint to finish. Above 0: start this long before the previous waypoint finishes, without waiting for any others. More than the previous one's whole time starts them together. A robot only does one waypoint at a time, so if the previous one is this robot's, this starts when that ends: any small amount keeps this robot going while other robots finish.")]
		public float OverlapFromLast = 1.736f;

		[DisplayName("Wait Start")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait before the move starts. The robot counts as controlled during it.")]
		public float WaitStart = 0.0f;

		[DisplayName("Wait End")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait after the move, and after the gripper finishes, before the next waypoint.")]
		public float WaitEnd = 0f;

		[Dropdown("None", "Close On Start", "Close On End", "Open On Start", "Open On End")]
		[Tooltip("When the gripper moves. On Start runs alongside the move. On End runs after the arm arrives, and the next waypoint waits for it.")]
		public int Gripper = 0;

		[DisplayName("Gripper Time")]
		[Units("s")]
		[Slider(0.0f, 5.0f)]
		[Tooltip("How long the gripper takes to open or close.")]
		public float GripperTime = 0.233f;

		[Dropdown("None", "Rest On Start", "Rest On End")]
		[Tooltip("Rest On Start: the robot goes to its rest pose if it isn't there already, then moves here. Rest On End: after this waypoint and its waits, the robot eases back to its rest pose. The director sets the rest pose.")]
		public int Rest = 0;

		[DisplayName("Rest Time")]
		[Units("s")]
		[Slider(0.1f, 10.0f)]
		[Tooltip("How long the robot takes to ease to its rest pose.")]
		public float RestTime = 0.6f;

		internal bool GripsOnStart => Gripper == CloseOnStart || Gripper == OpenOnStart;
		internal bool GripsOnEnd => Gripper == CloseOnEnd || Gripper == OpenOnEnd;
		internal float GripperOpening => Gripper == OpenOnStart || Gripper == OpenOnEnd ? 1.0f : 0.0f;

		/// <summary>The grasp site's world position this waypoint asks for.</summary>
		internal Vector3 HandPosition() => Transform.WorldTranslation;

		/// <summary>The unit direction to pull the elbow during this waypoint, or null if nothing is pulling it.</summary>
		internal Vector3? PoleDirection()
		{
			Entity? pole = UseElbowPole ? Array.Find(Children, child => child.Tag == ElbowPoleName) : null;
			if (pole is null)
				return null;
			Vector3 offset = pole.Transform.WorldTranslation - Transform.WorldTranslation;
			return offset.Length() > MinPoleOffset ? offset.Normalized() : null;
		}

		/// <summary>The grasp site's world rotation this waypoint asks for.</summary>
		internal Quaternion HandRotation() => (Transform.WorldRotationQuat * LD_Waypoint.MarkerToHandRotation).Normalized();
	}
}
