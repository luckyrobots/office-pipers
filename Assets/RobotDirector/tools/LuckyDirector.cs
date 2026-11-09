using System;
using Hazel;

namespace CanTake
{
	/// <summary>
	/// Every input the OpenArm pedestal motion graph exposes. A single-arm graph (Panda) has the same
	/// inputs without the LeftArm_/RightArm_ prefix; use the singleArm overloads for those. Each member is named exactly as the
	/// graph's identifier string, so <c>input.ID()</c> is what the RobotController's SetInput calls
	/// expect. RobotAction stores the index, so only ever append, and keep each RightArm_ member
	/// seven after its LeftArm_ twin (see <see cref="RobotInputs.ForArm"/>).
	/// </summary>
	public enum RobotInput
	{
		Start = 0,
		Duration = 1,

		LeftArm_Target_Position = 2,
		LeftArm_Target_Orientation = 3,
		LeftArm_Solve_Orientation = 4,
		LeftArm_Pole_Target = 5,
		LeftArm_Pole_Weight = 6,
		LeftArm_High_Accuracy = 7,
		LeftArm_Tracking_Mode = 8,

		RightArm_Target_Position = 9,
		RightArm_Target_Orientation = 10,
		RightArm_Solve_Orientation = 11,
		RightArm_Pole_Target = 12,
		RightArm_Pole_Weight = 13,
		RightArm_High_Accuracy = 14,
		RightArm_Tracking_Mode = 15,
	}

	public enum RobotInputKind { Trigger, Bool, Float, Vector3 }

	public static class RobotInputs
	{
		private const int ArmStride = RobotInput.RightArm_Target_Position - RobotInput.LeftArm_Target_Position;

		private static readonly Identifier[] s_IDs = Array.ConvertAll(Enum.GetNames<RobotInput>(), name => new Identifier(name));

		// The same inputs without the LeftArm_/RightArm_ prefix, as a single-arm graph (Panda) names them
		private static readonly Identifier[] s_SingleArmIDs = Array.ConvertAll(Enum.GetNames<RobotInput>(),
			name => new Identifier(name.StartsWith("LeftArm_") ? name.Substring(8) : name.StartsWith("RightArm_") ? name.Substring(9) : name));

		public static Identifier ID(this RobotInput input) => s_IDs[(int)input];

		public static RobotInputKind Kind(this RobotInput input) => input switch
		{
			RobotInput.Start => RobotInputKind.Trigger,
			RobotInput.Duration => RobotInputKind.Float,
			RobotInput.LeftArm_Pole_Weight or RobotInput.RightArm_Pole_Weight => RobotInputKind.Float,
			RobotInput.LeftArm_Target_Position or RobotInput.RightArm_Target_Position
				or RobotInput.LeftArm_Target_Orientation or RobotInput.RightArm_Target_Orientation
				or RobotInput.LeftArm_Pole_Target or RobotInput.RightArm_Pole_Target => RobotInputKind.Vector3,
			_ => RobotInputKind.Bool,
		};

		/// <summary>The given LeftArm_ input, or its RightArm_ twin when arm is 1.</summary>
		public static RobotInput ForArm(this RobotInput leftArmInput, int arm) => arm == 0 ? leftArmInput : leftArmInput + ArmStride;

		/// <summary>The input's identifier; on a single-arm robot either arm's input maps to the unprefixed name.</summary>
		public static Identifier ID(this RobotInput input, bool singleArm) => singleArm ? s_SingleArmIDs[(int)input] : s_IDs[(int)input];

		/// <summary>The given LeftArm_ input for an arm, as an identifier for this kind of robot.</summary>
		public static Identifier ForArm(this RobotInput leftArmInput, int arm, bool singleArm) => leftArmInput.ForArm(singleArm ? 0 : arm).ID(singleArm);

		/// <summary>True for a single-arm robot such as Panda, whose MuJoCo model has a plain "grasp_site".</summary>
		public static bool IsSingleArm(Entity robot)
		{
			MujocoSceneComponent? mujoco = robot.GetComponent<MujocoSceneComponent>();
			// GetSiteID is unsigned, so a missing site shows up as an index past the model's site count
			return mujoco is not null && mujoco.GetSiteID("grasp_site") < mujoco.ModelNsite;
		}

		/// <summary>True for a robot with a Pika gripper (Piper), told apart by its "pika_base" body.</summary>
		public static bool HasPikaHand(Entity robot)
		{
			MujocoSceneComponent? mujoco = robot.GetComponent<MujocoSceneComponent>();
			return mujoco is not null && mujoco.GetBodyID("pika_base") < mujoco.ModelNbody;
		}

		/// <summary>Holds an arm still at its hand's current position, so a Start pulse doesn't chase (0, 0, 0).</summary>
		public static void HoldArm(Entity robot, RobotControllerComponent controller, int arm, bool singleArm)
		{
			MujocoSceneComponent? mujoco = robot.GetComponent<MujocoSceneComponent>();
			if (mujoco is null)
				return;
			string site = singleArm ? "grasp_site" : arm == 0 ? "left_grasp_site" : "right_grasp_site";
			Vector3 mj = mujoco.GetSitePositionMj(mujoco.GetSiteID(site));
			Vector3 world = mujoco.GetAxisHzToMj().Conjugate * mj + mujoco.GetSceneTranslation();
			controller.SetInputBool(RobotInput.LeftArm_Solve_Orientation.ForArm(arm, singleArm), false);
			controller.SetInputVector3(RobotInput.LeftArm_Target_Position.ForArm(arm, singleArm), world);
		}
	}

	/// <summary>
	/// Plays a series of tasks on the OpenArm pedestal or a single-arm robot (Panda), one after
	/// another. Each child of Task List, in name order, is a waypoint that the hand moves to (an LD_Goal or
	/// LD_Target marker, which moves with LD_Waypoint's defaults, or an entity with LD_Waypoint), an LD_DuckGrabber (a waypoint that picks a random mesh inside a box), a CurvePath (the LD_Path prefab) that the hand follows once
	/// from end to end with CurveFollower's motion and its Path Task settings, an LD_Event that turns
	/// the hand and/or works the gripper in place (waiting for it, or Async), or a RobotAction.
	/// The script can be on the child itself or on one of its children. Each task runs, then
	/// the director waits for its hold time before starting the next. On Play both hands are first
	/// held where they are, so a move of one arm leaves the other still.
	/// </summary>
	public class RobotDirector : Entity
	{
		[Tooltip("The robot root entity, with the RobotController and MujocoScene components.")]
		public Entity? Robot;

		[Tooltip("An entity whose children are the tasks, played in name order: name them 1_Goal, 2_Path, 3_Target and so on (2 comes before 10). Each child is an LD_Goal, LD_Target or LD_Path instance, or an entity with a RobotAction, LD_Waypoint or CurvePath script (on it or a child).")]
		public Entity? TaskList;

		[Dropdown("Per Task", "Show All", "Hide All", "Hide All But Labels")]
		[Tooltip("While playing, whether the tasks are drawn. Per Task leaves each to its own setting (a curve's Visible During Sim; waypoint markers shown). Show All and Hide All set every curve and waypoint marker in the Task List; Hide All also hides the director's own logo. Hide All But Labels does the same and keeps each LD_DuckGrabber's Target Marker. Can be changed while playing.")]
		public int TaskVisibility = 2;

		private const int PerTask = 0;
		private const int ShowAll = 1;
		private const int HideAll = 2;
		private const int HideAllButLabels = 3;

		[ToggleButton("Loop", "Once")]
		[Tooltip("Start again from the first action after the last one finishes.")]
		public bool Loop = false;

		[ToggleButton("Tracking", "Seeded")]
		[Tooltip("Tracking starts each IK solve from the current pose, which keeps the elbow from flipping between moves. Seeded tries several starting poses each solve.")]
		public bool TrackingMode = true;

		[ToggleButton("High Accuracy", "Normal")]
		[Tooltip("Spend more solver iterations per update for a tighter fit to each target.")]
		public bool HighAccuracy = false;

		private RobotControllerComponent? m_Controller;
		private Entity?[] m_Tasks = Array.Empty<Entity?>(); // a RobotAction, LD_Waypoint, LD_Event, CurvePath or bare marker each, or null to skip
		private int m_Index = -1;
		private float m_Hold;
		private bool m_SingleArm;
		private Entity[] m_Entries = Array.Empty<Entity>(); // the Task List children, matching m_Tasks
		private CurveMotion? m_Curve;
		private bool m_CurveRunning;
		private float m_CurveWaitEnd;
		private int m_Pending = -1; // a task waiting out its Wait Start
		private int m_AppliedVisibility = -1; // none yet, so the first update always applies TaskVisibility
		private bool[] m_CurveVisibleDefaults = Array.Empty<bool>(); // each curve task's own Visible During Sim on Play, for Per Task
		private readonly System.Collections.Generic.List<PendingGrab> m_Grabs = new(); // grabs waiting out their Grabber Delay
		private readonly GripperRamp?[] m_Ramps = new GripperRamp?[2]; // each arm's gripper motion, held at its end value
		private readonly LuckyExporter.LD_DuckGrabber?[] m_Picks = new LuckyExporter.LD_DuckGrabber?[2]; // each arm's latest grabber task, whose mesh closing the gripper holds
		private readonly HeldMesh?[] m_Held = new HeldMesh?[2];
		private readonly bool[] m_HoldPending = new bool[2]; // the gripper is closing and hasn't taken hold yet
		private const double GripChangeEpsilon = 1e-3; // as a fraction of the gripper's range
		private readonly System.Collections.Generic.List<PendingRotate> m_Rotates = new(); // LD_Event wrist turns waiting their turn (steps of at most 90 degrees)
		private readonly System.Collections.Generic.List<AsyncTurn> m_AsyncTurns = new(); // Async LD_Event turns still under way, for the next move to carry
		private const float DegPerRad = 180.0f / MathF.PI;
		private const float MaxTurnStep = 90.0f; // a longer turn goes in steps, as each IK move takes the short way round
		private readonly Gripper?[] m_Grippers = new Gripper?[2];
		private readonly bool[] m_GripperLooked = new bool[2];
		private StreamedMove? m_Stream; // the waypoint move the director is driving itself, if any
		private const float MinMoveTime = 0.001f;
		private const float SettleTime = 0.05f; // the graph's blend when a streamed move sets off from rest

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
			// The Task List (and so Task Visibility) works even without a Robot, so the tasks can be
			// shown or hidden while the robot side is still being set up
			CollectTasks();
			ShowEventIcons(true);
			ApplyTaskVisibility();

			m_Controller = Robot?.GetComponent<RobotControllerComponent>();
			if (m_Controller is null)
			{
				Log.Warn($"[RobotDirector] {Tag}: set Robot to the robot root, with a RobotController component. Only Task Visibility works until then. (The LuckyDirector prefab once had a RobotDirector on both the LuckyDirector entity and its SM_Logo child; remove the one you don't use.)");
				return;
			}

			m_SingleArm = RobotInputs.IsSingleArm(Robot!);
			LuckyExporter.LD_Waypoint.PikaHand = RobotInputs.HasPikaHand(Robot!);
			for (int arm = 0; arm < (m_SingleArm ? 1 : 2); arm++)
			{
				RobotInputs.HoldArm(Robot!, m_Controller, arm, m_SingleArm);
				m_Controller.SetInputBool(RobotInput.LeftArm_Tracking_Mode.ForArm(arm, m_SingleArm), TrackingMode);
				m_Controller.SetInputBool(RobotInput.LeftArm_High_Accuracy.ForArm(arm, m_SingleArm), HighAccuracy);
			}

			Next();
		}

		private void CollectTasks()
		{
			Entity[] entries = Array.Empty<Entity>();
			if (TaskList is null)
				Log.Warn("[RobotDirector] Set Task List to an entity whose children are the tasks (1_Goal, 2_Path, ...).");
			else
			{
				// Hierarchy order doesn't follow the outliner, so the tasks go by name
				entries = TaskList.Children;
				Array.Sort(entries, (a, b) => NaturalCompare(a.Tag, b.Tag));
				if (entries.Length == 0)
					Log.Warn($"[RobotDirector] Task List ({TaskList.Tag}) has no children. Parent the LD_Goal, LD_Target or LD_Path instances under it.");
			}
			m_Entries = entries;
			m_Tasks = new Entity?[entries.Length];
			for (int i = 0; i < entries.Length; i++)
			{
				m_Tasks[i] = FindTask(entries[i]);
				if (m_Tasks[i] is null)
					Log.Warn($"[RobotDirector] Task {entries[i].Tag} has no mesh and no RobotAction, LD_Waypoint, LD_Event or CurvePath script, on it or its children, and will be skipped.");
				if (i > 0 && entries[i].Tag == entries[i - 1].Tag)
					Log.Warn($"[RobotDirector] Two tasks are both named '{entries[i].Tag}', so their order is arbitrary. Rename them, for example 1_{entries[i].Tag} and 2_{entries[i].Tag}.");
			}
			// With nothing to play, Loop would cycle through the empty entries forever in one frame
			if (!Array.Exists(m_Tasks, t => t is not null))
			{
				Log.Warn("[RobotDirector] No tasks to play: parent an LD_Goal, LD_Target or LD_Path under the Task List entity.");
				m_Tasks = Array.Empty<Entity?>();
				m_Entries = Array.Empty<Entity>();
			}

			m_CurveVisibleDefaults = new bool[m_Tasks.Length];
			for (int i = 0; i < m_Tasks.Length; i++)
				m_CurveVisibleDefaults[i] = m_Tasks[i]?.As<CurvePath>() is not CurvePath c || c.VisibleDuringSim;
		}

		protected override void OnUpdate(float deltaTime)
		{
			if (!StartOnce())
				return;
			if (TaskVisibility != m_AppliedVisibility)
				ApplyTaskVisibility();
			if (m_Controller is null)
				return;
			UpdateGrabs(deltaTime);
			UpdateRotates(deltaTime);
			UpdateStreamedMove(deltaTime);
			UpdateHeldMeshes(deltaTime);

			// A curve task runs until the hand reaches the end of the curve, then waits Wait End
			if (m_CurveRunning)
			{
				m_Curve!.Update(deltaTime);
				if (!m_Curve.Finished)
					return;
				m_CurveRunning = false;
				m_Hold = m_CurveWaitEnd;
			}
			if (m_Index >= m_Tasks.Length)
				return;

			m_Hold -= deltaTime;
			while (m_Hold <= 0.0f && !m_CurveRunning)
			{
				if (m_Pending >= 0)
				{
					int pending = m_Pending;
					m_Pending = -1;
					Run(pending);
				}
				else if (m_Index < m_Tasks.Length)
					Next();
				else
					break;
			}
		}

		// Sets each curve's Visible During Sim (CurvePath applies it on its next update) and each
		// waypoint marker's and event's meshes. Show All / Hide All override every task; Per Task puts
		// each curve back to its own Visible During Sim from Play and shows the markers and events.
		private void ApplyTaskVisibility()
		{
			m_AppliedVisibility = TaskVisibility;
			bool hideAll = TaskVisibility == HideAll || TaskVisibility == HideAllButLabels;
			bool perTask = TaskVisibility != ShowAll && !hideAll;
			bool show = !hideAll;
			for (int i = 0; i < m_Tasks.Length; i++)
			{
				Entity? task = m_Tasks[i];
				if (task is null || task.As<RobotAction>() is not null)
					continue;
				if (task.As<LuckyExporter.LD_DuckGrabber>() is LuckyExporter.LD_DuckGrabber grabber)
					grabber.MarkerHidden = TaskVisibility == HideAll;
				if (task.As<CurvePath>() is CurvePath curve)
					curve.VisibleDuringSim = perTask ? m_CurveVisibleDefaults[i] : show;
				else if (!show || task.As<LuckyExporter.LD_Event>() is null)
					// The whole Task List entry, as a marker's mesh can sit above or beside its script
					SetMeshesVisible(m_Entries[i], show);
			}
			if (show)
				ShowEventIcons(true); // each event shows only the icon for its Action

			// The director's own logo (SM_Logo, on this entity or under it) hides with Hide All too
			SetDirectorMeshesVisible(this, show);
		}

		// This entity's meshes and its children's, leaving out the Task List and Robot if they're
		// parented under the director (the tasks are handled above, and the robot is never hidden)
		private void SetDirectorMeshesVisible(Entity entity, bool visible)
		{
			if (entity.Id() == TaskList?.Id() || entity.Id() == Robot?.Id())
				return;
			StaticMeshComponent? mesh = entity.GetComponent<StaticMeshComponent>();
			if (mesh is not null)
				mesh.Visible = visible;
			foreach (Entity child in entity.Children)
				SetDirectorMeshesVisible(child, visible);
		}

		// Each LD_Event in the Task List shows only the icon for its Action
		private void ShowEventIcons(bool visible)
		{
			foreach (Entity? task in m_Tasks)
				if (task?.As<LuckyExporter.LD_Event>() is LuckyExporter.LD_Event ev)
					ev.ShowIcons(visible);
		}

		private static void SetMeshesVisible(Entity entity, bool visible)
		{
			StaticMeshComponent? mesh = entity.GetComponent<StaticMeshComponent>();
			if (mesh is not null)
				mesh.Visible = visible;
			foreach (Entity child in entity.Children)
				SetMeshesVisible(child, visible);
		}

		// Moves on to the next task, waiting its Wait Start first if it has one
		private void Next()
		{
			m_Index++;
			if (m_Index >= m_Tasks.Length)
			{
				if (!Loop || m_Tasks.Length == 0)
					return;
				m_Index = 0;
			}
			if (m_Tasks[m_Index] is null)
				return;

			float waitStart = WaitStart(m_Index);
			if (waitStart > 0.0f)
			{
				m_Pending = m_Index;
				m_Hold += waitStart;
			}
			else
				Run(m_Index);
		}

		private float WaitStart(int index)
		{
			Entity task = m_Tasks[index]!;
			if (task.As<LuckyExporter.LD_Waypoint>() is LuckyExporter.LD_Waypoint waypoint)
				return waypoint.WaitStart;
			if (task.As<LuckyExporter.LD_Event>() is LuckyExporter.LD_Event ev)
				return ev.WaitStart;
			if (task.As<CurvePath>() is CurvePath path)
				return path.WaitStart;
			return 0.0f;
		}

		private void Run(int index)
		{
			Entity task = m_Tasks[index]!;
			// Whatever runs next owns the arm's target, so the last streamed move stops writing it
			StreamedMove? previous = m_Stream;
			m_Stream = null;
			if (task.As<RobotAction>() is RobotAction action)
				m_Hold += action.Run(m_Controller!, m_SingleArm);
			else if (task.As<LuckyExporter.LD_Waypoint>() is LuckyExporter.LD_Waypoint waypoint)
			{
				// A grabber with nothing to go for still takes its time, so a Loop of them can't spin forever in one update
				LuckyExporter.LD_DuckGrabber? grabber = waypoint as LuckyExporter.LD_DuckGrabber;
				if (grabber is not null)
				{
					if (!grabber.PickTarget(Robot, TaskList, this))
					{
						m_Hold += MathF.Max(waypoint.Duration, MinMoveTime) + waypoint.Hold;
						return;
					}
					m_Picks[ArmIndex(waypoint.Arm)] = grabber;
				}
				ScheduleGrab(waypoint.GrabberAction, waypoint.GrabberValue, waypoint.GrabberDelay, waypoint.GrabberTime, waypoint.Arm);
				Quaternion? hand = CarryAsyncTurn(waypoint.Arm, () => waypoint.SolveOrientation ? waypoint.HandRotation() : CurrentHand(waypoint.Arm));
				LuckyExporter.LD_Waypoint? next = ImmediateAfter(index, waypoint);
				(next as LuckyExporter.LD_DuckGrabber)?.AimAtSharedTarget(); // the speed through to it depends on where it really is
				// A hand following a mesh needs its target rewritten every update, so the director drives that too
				bool follow = grabber is not null && grabber.Follows;
				if ((waypoint.Immediate || next is not null || follow) && StartStreamedMove(waypoint, next, hand, previous))
				{
					if (follow)
					{
						m_Stream!.Following = grabber;
						m_Stream.FollowTurn = hand.HasValue ? (waypoint.HandRotation().Conjugate * hand.Value).Normalized() : null;
					}
					m_Hold += waypoint.Duration + waypoint.Hold;
				}
				else
					m_Hold += waypoint.Run(m_Controller!, m_SingleArm, hand) + waypoint.Hold;
				UpdateGrabs(0.0f); // a zero delay grabs as the move starts
			}
			else if (task.As<LuckyExporter.LD_Event>() is LuckyExporter.LD_Event ev)
			{
				// Everything is scheduled on its own timer; Wait holds the tasks until it has all finished
				float busy = 0.0f;
				float turnTime = ev.RotateTime;
				if (turnTime > 0.0f)
				{
					int steps = (int)MathF.Ceiling(MathF.Abs(ev.Amount) / MaxTurnStep);
					for (int step = 0; step < steps; step++)
						m_Rotates.Add(new PendingRotate(step * turnTime / steps, turnTime / steps, ev.Arm, ev.Amount / steps));
					busy = turnTime;
					if (ev.IsAsync)
						m_AsyncTurns.Add(new AsyncTurn(ArmIndex(ev.Arm), ev.Amount, turnTime));
				}
				else if (ScheduleGrab(ev.GrabberAction, ev.Amount, 0.0f, ev.Time, ev.Arm))
					busy = MathF.Max(ev.Time, 0.0f);
				m_Hold += (ev.IsAsync ? 0.0f : busy) + ev.Hold;
				UpdateGrabs(0.0f);
				UpdateRotates(0.0f);
			}
			else if (task.As<CurvePath>() is CurvePath curve)
				StartCurve(curve);
			else
			{
				// A bare LD_Goal / LD_Target marker
				Quaternion? hand = CarryAsyncTurn(0, () => LuckyExporter.LD_Waypoint.DefaultHandRotation(task));
				m_Hold += LuckyExporter.LD_Waypoint.RunDefault(task, m_Controller!, m_SingleArm, hand);
			}
		}

		private int ArmIndex(int arm) => m_SingleArm ? 0 : arm;

		// A straight waypoint move the director drives by writing the target every update. The graph's
		// own move (Start) eases to a stop at both ends, but between moves the hand follows the target
		// directly, so this can pass through a waypoint without stopping.
		private sealed class StreamedMove
		{
			public int Arm;
			public Vector3 From;
			public Vector3 To;
			public Quaternion? RotationFrom; // null with Position Only
			public Quaternion? RotationTo;
			public float Time;
			public float Elapsed;
			public float StartSpeed; // metres per second, along the line
			public float EndSpeed;
			public bool Done;
			public LuckyExporter.LD_DuckGrabber? Following; // re-aims the move at this grabber's mesh until the gripper closes
			public Quaternion? FollowTurn; // an Async turn this move carries, kept while following
		}

		// The next task, if it's an Immediate waypoint this one should run straight into without stopping
		private LuckyExporter.LD_Waypoint? ImmediateAfter(int index, LuckyExporter.LD_Waypoint waypoint)
		{
			if (waypoint.Hold > 0.0f)
				return null;
			for (int step = 1; step <= m_Tasks.Length; step++)
			{
				if (index + step >= m_Tasks.Length && !Loop)
					return null;
				Entity? task = m_Tasks[(index + step) % m_Tasks.Length];
				if (task is null)
					continue; // skipped when played
				return task.As<LuckyExporter.LD_Waypoint>() is LuckyExporter.LD_Waypoint next
					&& next.Immediate && next.WaitStart <= 0.0f && ArmIndex(next.Arm) == ArmIndex(waypoint.Arm) ? next : null;
			}
			return null;
		}

		// Starts driving the hand to a waypoint: from the previous move's end if that was left moving,
		// otherwise from rest at the hand's current pose. False if the hand's pose can't be read.
		private bool StartStreamedMove(LuckyExporter.LD_Waypoint waypoint, LuckyExporter.LD_Waypoint? next, Quaternion? hand, StreamedMove? previous)
		{
			int arm = ArmIndex(waypoint.Arm);
			bool carried = previous is not null && previous.Arm == arm && previous.EndSpeed > 0.0f;
			Vector3 from;
			Quaternion current;
			if (carried)
			{
				from = previous!.To;
				current = previous.RotationTo ?? CurrentHand(arm) ?? Quaternion.Identity;
			}
			else if (!TryGetHand(arm, out from, out current))
				return false;

			Vector3 to = waypoint.Transform.WorldTranslation;
			bool solve = waypoint.SolveOrientation || hand.HasValue;
			float time = MathF.Max(waypoint.Duration, MinMoveTime);
			float speed = (to - from).Length() / time;

			// The speed through the next waypoint is the average of the two moves' own, limited so that
			// neither move overshoots its end and comes back (3x its average is the most that allows)
			float endSpeed = 0.0f;
			if (next is not null)
			{
				float nextSpeed = (next.Transform.WorldTranslation - to).Length() / MathF.Max(next.Duration, MinMoveTime);
				endSpeed = MathF.Min((speed + nextSpeed) * 0.5f, 3.0f * MathF.Min(speed, nextSpeed));
			}

			m_Stream = new StreamedMove
			{
				Arm = arm,
				From = from,
				To = to,
				RotationFrom = solve ? current : null,
				RotationTo = solve ? hand ?? waypoint.HandRotation() : null,
				Time = time,
				Elapsed = MathF.Max(-m_Hold, 0.0f), // the part of this update after the last task ended
				StartSpeed = carried ? previous!.EndSpeed : 0.0f,
				EndSpeed = endSpeed,
			};
			m_Controller!.SetInputBool(RobotInput.LeftArm_Solve_Orientation.ForArm(arm, m_SingleArm), solve);
			WriteStreamedMove(m_Stream);

			// From rest the graph may still be blending (or never have started, on the first task), so
			// settle it onto the hand. A move carried on from another is already following the target.
			if (!carried)
			{
				m_Controller.SetInputFloat(RobotInput.Duration.ID(), SettleTime);
				m_Controller.SetInputTrigger(RobotInput.Start.ID());
			}
			return true;
		}

		private void UpdateStreamedMove(float deltaTime)
		{
			if (m_Stream is null)
				return;
			bool follow = m_Stream.Following is not null && m_Stream.Following.StillFollowing;
			if (m_Stream.Done && !follow)
				return;
			if (follow)
			{
				LuckyExporter.LD_DuckGrabber grabber = m_Stream.Following!;
				grabber.FollowCurrentPose();
				m_Stream.To = grabber.Transform.WorldTranslation;
				if (m_Stream.RotationTo.HasValue)
					m_Stream.RotationTo = m_Stream.FollowTurn is Quaternion turn ? (grabber.HandRotation() * turn).Normalized() : grabber.HandRotation();
			}
			m_Stream.Elapsed += deltaTime;
			WriteStreamedMove(m_Stream);
		}

		private const float MinCentreTime = 0.1f;
		private const float HoldReach = 0.04f; // how near the grip point the hand must be for closing to hold the mesh

		// A mesh locked to a hand, as its pose relative to the grasp site when the gripper started closing
		private sealed class HeldMesh
		{
			public uint Body;
			public Vector3 Position; // as taken hold of
			public Vector3 Centred; // with its grip point between the fingers, where it's drawn to as they close
			public Quaternion Rotation;
			public float Elapsed;
			public float CentreTime;
		}

		// Holds the mesh this arm last went for, once the hand has reached it: the hand trails its target, so
		// it can still be arriving as the gripper starts closing. Called until the gripper finishes closing.
		// A free body's transform comes from the physics, so parenting its entity wouldn't move it: it's
		// welded to a pose instead, which UpdateHeldMeshes moves with the hand. Scene meshes' bodies are
		// named from their entity ID.
		private void TryHold(int arm, bool lastTry)
		{
			LuckyExporter.LD_DuckGrabber? pick = m_Picks[arm];
			if (pick is null || !pick.HoldsTarget || pick.Target is not Entity target || !target || m_Held[arm] is not null)
			{
				m_HoldPending[arm] = false;
				return;
			}
			MujocoSceneComponent? mujoco = Robot!.GetComponent<MujocoSceneComponent>();
			if (mujoco is null || !TryGetHand(arm, out Vector3 handWorld, out Quaternion _))
				return;
			float distance = (handWorld - pick.GripWorld).Length();
			if (distance > HoldReach)
			{
				m_HoldPending[arm] = !lastTry;
				if (lastTry)
					Log.Warn($"[RobotDirector] {pick.Tag}: the hand closed {distance:0.000} m from {target.Tag}'s grip point, so it isn't holding it.");
				return;
			}
			m_HoldPending[arm] = false;
			uint body = mujoco.GetBodyID($"lucky_body_{target.Id()}");
			if (body >= mujoco.ModelNbody || !TryGetHandMj(mujoco, arm, out Vector3 handPosition, out Quaternion handRotation)
				|| !mujoco.GetBodyPoseMj(body, out Vector3 position, out Quaternion rotation, out Vector3 _, out Vector3 _))
			{
				Log.Warn($"[RobotDirector] {pick.Tag}: {target.Tag} has no physics body to hold.");
				return;
			}
			Quaternion toHand = handRotation.Conjugate;
			Vector3 held = toHand * (position - handPosition);
			Vector3 gripMj = mujoco.GetAxisHzToMj() * (pick.GripWorld - mujoco.GetSceneTranslation());
			m_Held[arm] = new HeldMesh
			{
				Body = body,
				Position = held,
				Centred = held - toHand * (gripMj - handPosition),
				Rotation = (toHand * rotation).Normalized(),
				CentreTime = MathF.Max(m_Ramps[arm]?.Time ?? 0.0f, MinCentreTime),
			};
		}

		private void ReleaseHold(int arm)
		{
			m_HoldPending[arm] = false;
			if (m_Held[arm] is not HeldMesh held)
				return;
			Robot!.GetComponent<MujocoSceneComponent>()?.UnweldBody(held.Body);
			m_Held[arm] = null;
			m_Picks[arm] = null; // done with this mesh, so a later close doesn't go back to it
		}

		private void UpdateHeldMeshes(float deltaTime)
		{
			for (int arm = 0; arm < m_Held.Length; arm++)
			{
				if (m_HoldPending[arm])
					TryHold(arm, m_Ramps[arm] is not GripperRamp ramp || ramp.Elapsed >= ramp.Time);
				if (m_Held[arm] is not HeldMesh held)
					continue;
				MujocoSceneComponent? mujoco = Robot!.GetComponent<MujocoSceneComponent>();
				held.Elapsed += deltaTime;
				float t = Math.Clamp(held.Elapsed / held.CentreTime, 0.0f, 1.0f);
				Vector3 inHand = held.Position + (held.Centred - held.Position) * (t * t * (3.0f - 2.0f * t));
				if (mujoco is not null && TryGetHandMj(mujoco, arm, out Vector3 position, out Quaternion rotation))
					mujoco.WeldBody(held.Body, position + rotation * inHand, (rotation * held.Rotation).Normalized(), true);
			}
		}

		private bool TryGetHandMj(MujocoSceneComponent mujoco, int arm, out Vector3 position, out Quaternion rotation)
		{
			string site = m_SingleArm ? "grasp_site" : arm == 0 ? "left_grasp_site" : "right_grasp_site";
			return mujoco.GetSitePoseMj(mujoco.GetSiteID(site), out position, out rotation, out Vector3 _);
		}

		// Sets the arm's target for the move's elapsed time. The distance along the line is a cubic with
		// the move's start and end speeds, so the speed carries across from one move to the next.
		private void WriteStreamedMove(StreamedMove move)
		{
			float u = Math.Clamp(move.Elapsed / move.Time, 0.0f, 1.0f);
			move.Done = u >= 1.0f;
			float u2 = u * u;
			float u3 = u2 * u;
			float ease = 3.0f * u2 - 2.0f * u3;
			float length = (move.To - move.From).Length();
			float distance = ease * length
				+ (u3 - 2.0f * u2 + u) * move.Time * move.StartSpeed
				+ (u3 - u2) * move.Time * move.EndSpeed;
			float fraction = length > 1e-5f ? distance / length : ease; // a move with no distance can still turn the hand

			m_Controller!.SetInputVector3(RobotInput.LeftArm_Target_Position.ForArm(move.Arm, m_SingleArm), move.From + (move.To - move.From) * fraction);
			if (move.RotationFrom is Quaternion from && move.RotationTo is Quaternion to)
				m_Controller.SetInputVector3(RobotInput.LeftArm_Target_Orientation.ForArm(move.Arm, m_SingleArm),
					Quaternion.Slerp(from, to, fraction).Normalized().EulerAngles * DegPerRad);
		}

		// An Async LD_Event turn: its whole Amount, and how long it has left to run on its own
		private sealed class AsyncTurn
		{
			public readonly int Arm;
			public readonly float Degrees;
			public float TimeLeft;
			public AsyncTurn(int arm, float degrees, float timeLeft) { Arm = arm; Degrees = degrees; TimeLeft = timeLeft; }
		}

		// A move starting on an arm takes the arm's IK over, so it carries any Async turn still under way
		// there: the hand ends at the move's own orientation (from orientation()) turned by the turns'
		// Amount about the fingers' axis, and the turn's remaining steps are dropped. null when there's
		// no such turn, and the move runs as it is.
		private Quaternion? CarryAsyncTurn(int arm, Func<Quaternion?> orientation)
		{
			arm = ArmIndex(arm);
			float degrees = 0.0f;
			for (int i = 0; i < m_AsyncTurns.Count; i++)
			{
				if (m_AsyncTurns[i].Arm != arm)
					continue;
				degrees += m_AsyncTurns[i].Degrees;
				m_AsyncTurns.RemoveAt(i--);
			}
			if (degrees == 0.0f)
				return null;
			m_Rotates.RemoveAll(turn => ArmIndex(turn.Arm) == arm);
			if (orientation() is not Quaternion hand)
				return null;
			return (hand * Quaternion.AngleAxis(degrees, LuckyExporter.LD_Waypoint.FingerAxis)).Normalized();
		}

		// The grasp site's world position and rotation for an arm, from the physics; false without one
		private bool TryGetHand(int arm, out Vector3 position, out Quaternion rotation)
		{
			position = Vector3.Zero;
			rotation = Quaternion.Identity;
			MujocoSceneComponent? mujoco = Robot!.GetComponent<MujocoSceneComponent>();
			if (mujoco is null)
				return false;
			string site = m_SingleArm ? "grasp_site" : arm == 0 ? "left_grasp_site" : "right_grasp_site";
			if (!mujoco.GetSitePoseMj(mujoco.GetSiteID(site), out Vector3 positionMj, out Quaternion rotationMj, out Vector3 _))
				return false;
			Quaternion mjToHz = mujoco.GetAxisHzToMj().Conjugate;
			position = mjToHz * positionMj + mujoco.GetSceneTranslation();
			rotation = mjToHz * rotationMj;
			return true;
		}

		private Quaternion? CurrentHand(int arm) => TryGetHand(ArmIndex(arm), out _, out Quaternion rotation) ? rotation : null;

		// A gripper actuator and its open and closed control values
		private sealed class Gripper
		{
			public MujocoActuator Actuator = null!;
			public double Open;
			public double Closed;
		}

		// Gripper actuator names to look for: Panda, Piper, then the OpenArm pedestal's per-arm fingers
		private static readonly string[] s_SingleArmGrippers = { "gripper", "actuator8" };
		private static readonly string[][] s_ArmGrippers = { new[] { "left_finger1_ctrl", "left_gripper" }, new[] { "right_finger1_ctrl", "right_gripper" } };

		// Queues a gripper motion for a Grabber Action (LD_Waypoint's Open, Close or Set); false for None
		private bool ScheduleGrab(int action, float value, float delay, float time, int arm)
		{
			float? opening = action switch
			{
				LuckyExporter.LD_Waypoint.GrabOpen => 1.0f,
				LuckyExporter.LD_Waypoint.GrabClose => 0.0f,
				LuckyExporter.LD_Waypoint.GrabSet => Math.Clamp(value, 0.0f, 1.0f),
				_ => null,
			};
			if (opening is not float amount)
				return false;
			m_Grabs.Add(new PendingGrab(delay, time, arm, amount));
			return true;
		}

		private sealed class PendingRotate
		{
			public float Delay;
			public readonly float Time;
			public readonly int Arm;
			public readonly float Degrees;
			public PendingRotate(float delay, float time, int arm, float degrees) { Delay = delay; Time = time; Arm = arm; Degrees = degrees; }
		}

		private void UpdateRotates(float deltaTime)
		{
			for (int i = 0; i < m_AsyncTurns.Count; i++)
			{
				m_AsyncTurns[i].TimeLeft -= deltaTime;
				if (m_AsyncTurns[i].TimeLeft <= 0.0f)
					m_AsyncTurns.RemoveAt(i--); // finished on its own
			}
			for (int i = 0; i < m_Rotates.Count; i++)
			{
				PendingRotate turn = m_Rotates[i];
				turn.Delay -= deltaTime;
				if (turn.Delay > 0.0f)
					continue;
				m_Rotates.RemoveAt(i--);
				StartRotate(turn);
			}
		}

		// Turns the hand about the fingers' axis (the grasp site's Z) from where it is now, keeping its
		// position: an IK move to the same place with the orientation turned, eased over the step's time
		private void StartRotate(PendingRotate turn)
		{
			int arm = ArmIndex(turn.Arm);
			if (!TryGetHand(arm, out Vector3 position, out Quaternion hand))
				return;
			Quaternion turned = (hand * Quaternion.AngleAxis(turn.Degrees, LuckyExporter.LD_Waypoint.FingerAxis)).Normalized();
			m_Controller!.SetInputBool(RobotInput.LeftArm_Solve_Orientation.ForArm(arm, m_SingleArm), true);
			m_Controller.SetInputVector3(RobotInput.LeftArm_Target_Orientation.ForArm(arm, m_SingleArm), turned.EulerAngles * DegPerRad);
			m_Controller.SetInputVector3(RobotInput.LeftArm_Target_Position.ForArm(arm, m_SingleArm), position);
			m_Controller.SetInputFloat(RobotInput.Duration.ID(), MathF.Max(turn.Time, 0.05f));
			m_Controller.SetInputTrigger(RobotInput.Start.ID());
		}

		private sealed class PendingGrab
		{
			public float Delay;
			public readonly float Time;
			public readonly int Arm;
			public readonly float Opening; // 0 closed .. 1 open
			public PendingGrab(float delay, float time, int arm, float opening) { Delay = delay; Time = time; Arm = arm; Opening = opening; }
		}

		private sealed class GripperRamp
		{
			public Gripper Gripper = null!;
			public double From;
			public double To;
			public float Time;
			public float Elapsed;
			public float StartGap; // the fingers' distance when this motion started, 0 if it can't be read
			public double StartOpening; // and the gripper's opening then, 0 closed .. 1 open
			public float LastGap;
			public bool Settled; // eased off to a firm grip on a held mesh
		}

		// Runs apart from the task sequence: counts down each scheduled grab, then moves each arm's
		// gripper towards its target over Grabber Time. The value is written every update, also once
		// it has arrived, so nothing else can take the gripper back while the arm moves.
		private void UpdateGrabs(float deltaTime)
		{
			for (int i = 0; i < m_Grabs.Count; i++)
			{
				PendingGrab grab = m_Grabs[i];
				grab.Delay -= deltaTime;
				if (grab.Delay > 0.0f)
					continue;
				m_Grabs.RemoveAt(i--);
				StartGrab(grab, -grab.Delay); // the part of this update after the delay ran out
			}

			for (int arm = 0; arm < m_Ramps.Length; arm++)
			{
				GripperRamp? ramp = m_Ramps[arm];
				if (ramp is null)
					continue;
				ramp.Elapsed += deltaTime;
				float t = ramp.Time <= 0.0f ? 1.0f : Math.Clamp(ramp.Elapsed / ramp.Time, 0.0f, 1.0f);
				t = t * t * (3.0f - 2.0f * t); // ease in and out
				ramp.Gripper.Actuator.IK = false;
				ramp.Gripper.Actuator.Value = ramp.From + (ramp.To - ramp.From) * t;
				StopAtContact(arm, ramp);
			}
		}

		// With a mesh held, a full squeeze twists it against its weld, so the gripper eases off to a firm grip
		// once the mesh has been drawn between the fingers and they've stopped moving against it.
		private void StopAtContact(int arm, GripperRamp ramp)
		{
			if (ramp.Settled || m_Held[arm] is not HeldMesh held || ramp.StartGap <= 0.0f || !TryGetFingerGap(out float gap))
				return;
			double range = ramp.Gripper.Open - ramp.Gripper.Closed;
			bool still = MathF.Abs(gap - ramp.LastGap) < FingersStill;
			ramp.LastGap = gap;
			if ((ramp.To - ramp.From) / range >= 0.0 || held.Elapsed < held.CentreTime || !still)
				return;
			double commanded = (ramp.Gripper.Actuator.Value - ramp.Gripper.Closed) / range;
			double actual = ramp.StartOpening * gap / ramp.StartGap;
			if (actual - commanded <= GripSqueeze)
				return;
			double firm = ramp.Gripper.Closed + range * (actual - GripSqueeze);
			ramp.From = ramp.To = firm;
			ramp.Gripper.Actuator.Value = firm;
			ramp.Settled = true;
		}

		private const double GripSqueeze = 0.06; // how far past the fingers the gripper is told to close, as a fraction of its range: about 5 mm on Panda
		private const float FingersStill = 0.0002f; // metres between updates

		// The distance between Panda's fingers; false on a robot without bodies of these names
		private bool TryGetFingerGap(out float gap)
		{
			gap = 0.0f;
			MujocoSceneComponent? mujoco = Robot!.GetComponent<MujocoSceneComponent>();
			if (mujoco is null)
				return false;
			uint left = mujoco.GetBodyID("left_finger");
			uint right = mujoco.GetBodyID("right_finger");
			if (left >= mujoco.ModelNbody || right >= mujoco.ModelNbody
				|| !mujoco.GetBodyPoseMj(left, out Vector3 a, out Quaternion _, out Vector3 _, out Vector3 _)
				|| !mujoco.GetBodyPoseMj(right, out Vector3 b, out Quaternion _, out Vector3 _, out Vector3 _))
				return false;
			gap = (a - b).Length();
			return true;
		}

		// Starts moving an arm's gripper, from wherever it is now; replaces any motion already under way
		private void StartGrab(PendingGrab grab, float elapsed)
		{
			int arm = m_SingleArm ? 0 : grab.Arm;
			Gripper? gripper = FindGripper(arm);
			if (gripper is null)
				return;
			double from = gripper.Actuator.Value;
			double to = gripper.Closed + (gripper.Open - gripper.Closed) * grab.Opening;
			TryGetFingerGap(out float startGap);
			m_Ramps[arm] = new GripperRamp
			{
				Gripper = gripper,
				From = from,
				To = to,
				Time = grab.Time,
				Elapsed = elapsed,
				StartGap = startGap,
				StartOpening = (from - gripper.Closed) / (gripper.Open - gripper.Closed),
			};

			// Which way this goes in terms of opening, whichever way round the actuator's values run
			double range = gripper.Open - gripper.Closed;
			double change = range != 0.0 ? (to - from) / range : 0.0;
			if (change < -GripChangeEpsilon)
			{
				// From here the mesh moves with the hand, and afterwards it falls: following would chase it
				m_Picks[arm]?.StopFollowing();
				m_HoldPending[arm] = true;
			}
			else if (change > GripChangeEpsilon)
				ReleaseHold(arm);
		}

		private Gripper? FindGripper(int arm)
		{
			if (m_GripperLooked[arm])
				return m_Grippers[arm];
			m_GripperLooked[arm] = true;
			MujocoSceneComponent? mujoco = Robot!.GetComponent<MujocoSceneComponent>();
			string[] names = m_SingleArm ? s_SingleArmGrippers : s_ArmGrippers[arm];
			foreach (string name in names)
			{
				MujocoActuator? actuator = mujoco is not null && mujoco.GetActuatorID(name) >= 0 ? mujoco.GetActuator(name) : null;
				if (actuator is null)
					continue;
				// Open is the end of the control range furthest from zero: 255 on Panda (0 closed),
				// 0.035 m on Piper, the finger's full angle on OpenArm
				var gripper = new Gripper { Actuator = actuator, Open = 255.0, Closed = 0.0 };
				if (mujoco!.GetActuatorCtrlRange(mujoco.GetActuatorIndex(name), out float low, out float high) && high > low)
				{
					// The Pika's jaws are the exception: its slide is fully open at 0 and closes towards the far end
					bool highIsOpen = RobotInputs.HasPikaHand(Robot!) ? Math.Abs(high) <= Math.Abs(low) : Math.Abs(high) >= Math.Abs(low);
					gripper.Open = highIsOpen ? high : low;
					gripper.Closed = highIsOpen ? low : high;
				}
				else if (name == "gripper")
					gripper.Open = 0.035; // Piper's finger slide, in metres
				// Script-owned, so the motion graph doesn't overwrite it every step
				gripper.Actuator.IK = false;
				m_Grippers[arm] = gripper;
				return gripper;
			}
			Log.Warn($"[RobotDirector] Grabber Action: no gripper actuator found on {Robot!.Tag} (looked for {string.Join(", ", names)}).");
			return null;
		}

		private void StartCurve(CurvePath curve)
		{
			m_Curve ??= new CurveMotion(Robot!, m_Controller!, m_SingleArm);
			curve.ApplyTask(m_Curve);
			m_Curve.HighAccuracy = HighAccuracy;
			m_Curve.TrackingMode = TrackingMode;
			m_Curve.Once = true;
			m_Curve.Begin(curve);
			m_CurveRunning = true;
			m_CurveWaitEnd = curve.WaitEnd;
			m_Hold = 0.0f;
			ScheduleGrab(curve.GrabberAction, curve.GrabberValue, curve.GrabberDelay, curve.GrabberTime, curve.Arm);
			UpdateGrabs(0.0f); // a zero delay grabs as the approach starts
		}

		// Compares names so that Task_2 comes before Task_10
		internal static int NaturalCompare(string a, string b)
		{
			int i = 0, j = 0;
			while (i < a.Length && j < b.Length)
			{
				if (char.IsDigit(a[i]) && char.IsDigit(b[j]))
				{
					int si = i, sj = j;
					while (i < a.Length && char.IsDigit(a[i])) i++;
					while (j < b.Length && char.IsDigit(b[j])) j++;
					int cmp = long.Parse(a[si..i]).CompareTo(long.Parse(b[sj..j]));
					if (cmp != 0)
						return cmp;
				}
				else
				{
					int cmp = char.ToLowerInvariant(a[i]).CompareTo(char.ToLowerInvariant(b[j]));
					if (cmp != 0)
						return cmp;
					i++;
					j++;
				}
			}
			return (a.Length - i).CompareTo(b.Length - j);
		}

		// The entity, or the nearest child below it, that carries a task script (LD_Path keeps its
		// CurvePath on a child). Failing that, the entry itself if it has a mesh: a bare LD_Goal or
		// LD_Target marker, which moves with LD_Waypoint's defaults.
		private static Entity? FindTask(Entity entry)
		{
			var queue = new System.Collections.Generic.Queue<Entity>();
			queue.Enqueue(entry);
			while (queue.Count > 0)
			{
				Entity e = queue.Dequeue();
				if (e.As<RobotAction>() is not null || e.As<CurvePath>() is not null || e.As<LuckyExporter.LD_Waypoint>() is not null
					|| e.As<LuckyExporter.LD_Event>() is not null)
					return e;
				foreach (Entity child in e.Children)
					queue.Enqueue(child);
			}
			return entry.GetComponent<StaticMeshComponent>() is not null ? entry : null;
		}
	}
}
