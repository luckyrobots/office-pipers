using System;
using Hazel;

namespace LuckyExporter
{
	/// <summary>
	/// A task that works the hand where it is, without moving the arm: turns the end of the arm about
	/// the fingers' axis (Rotate), or opens, closes or sets the gripper. Put it in a RobotDirector's
	/// Task List (the LD_Event prefab). With Wait, the director waits for the action to finish, then
	/// Wait End. With Async, the next task starts after Wait End while the action plays out: the
	/// gripper moves on its own, and a turn still under way is added to the next waypoint's move, so
	/// the hand turns as it travels.
	///
	/// Its child icons show the Action: LD_Rotate for Rotate, LD_Open for Open Grabber, LD_Close for
	/// Close Grabber and Set Grabber. The action starts after Wait Start and takes Time. LuckyLive keeps them in step in Edit mode (luckylive/ld_event.py),
	/// and the director sets them when Play starts.
	///
	/// The rotation is an IK move to the hand's current position with its orientation turned. Added
	/// to a waypoint's move, it turns the hand by Amount from that waypoint's orientation, the short
	/// way (up to 180 degrees), over the move's Duration.
	/// LuckyEngine's prefab editor can't load project scripts and may crash on the LD_Event prefab:
	/// adjust the instances in the scene.
	/// </summary>
	public class LD_Event : Entity
	{
		internal const int ActionRotate = 0;
		internal const int ActionOpen = 1;
		internal const int ActionClose = 2;
		internal const int ActionSet = 3;

		// The child icon for each Action, by tag
		internal const string RotateIcon = "LD_Rotate";
		internal const string OpenIcon = "LD_Open";
		internal const string CloseIcon = "LD_Close";

		private const float MinRotateTime = 0.05f;

		[Dropdown("Left Arm", "Right Arm")]
		[Tooltip("Which hand this works. Ignored on a single-arm robot (Panda).")]
		public int Arm = 0;

		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait before this event starts.")]
		public float WaitStart = 0.0f;

		[DisplayName("Wait End")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait after the event (or, with Async, after it starts) before the next task.")]
		public float Hold = 0f;

		[Dropdown("Rotate", "Open Grabber", "Close Grabber", "Set Grabber")]
		[Tooltip("Rotate turns the end of the arm about the fingers' axis by Amount degrees, keeping the hand where it is. Open and Close Grabber open or close the gripper fully; Set Grabber moves it to Amount (0 closed to 1 fully open).")]
		public int Action = 1;

		[Tooltip("Rotate: degrees to turn about the fingers' axis, negative the other way (joint limits still apply, about 166 degrees each way at the Panda's wrist). Set Grabber: how far open, from 0 (closed) to 1 (fully open), on any robot's gripper range. Open and Close Grabber don't use it.")]
		public float Amount = 90.0f;

		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("How long the turn or the gripper motion takes, starting after Wait Start. A gripper with 0 snaps.")]
		public float Time = 0.394f;

		[DisplayName("Is Async")]
		[ToggleButton("Async", "Wait")]
		[Tooltip("Async: the next task starts with this event (after Wait End) while the action plays out. The gripper moves on its own; a turn still under way is added to the next waypoint's move, so the hand turns as it travels. Wait: the next task waits until the action has finished, then Wait End.")]
		public bool IsAsync = false;

		/// <summary>The rotation's length in seconds, or 0 when there's nothing to turn.</summary>
		internal float RotateTime => Action == ActionRotate && Amount != 0.0f ? MathF.Max(Time, MinRotateTime) : 0.0f;

		/// <summary>The gripper motion as an LD_Waypoint Grabber Action.</summary>
		internal int GrabberAction => Action switch
		{
			ActionOpen => LD_Waypoint.GrabOpen,
			ActionClose => LD_Waypoint.GrabClose,
			ActionSet => LD_Waypoint.GrabSet,
			_ => LD_Waypoint.GrabNone,
		};

		/// <summary>The child icon tag that shows this event's Action.</summary>
		internal static string IconFor(int action) => action switch
		{
			ActionOpen => OpenIcon,
			ActionClose or ActionSet => CloseIcon,
			_ => RotateIcon,
		};

		/// <summary>Shows the icon for the Action and hides the other two (children tagged LD_Rotate, LD_Open, LD_Close).</summary>
		internal void ShowIcons(bool visible = true)
		{
			string wanted = IconFor(Action);
			foreach (Entity child in Children)
			{
				string tag = child.Tag;
				if (tag != RotateIcon && tag != OpenIcon && tag != CloseIcon)
					continue;
				StaticMeshComponent? mesh = child.GetComponent<StaticMeshComponent>();
				if (mesh is not null)
					mesh.Visible = visible && tag == wanted;
			}
		}
	}
}
