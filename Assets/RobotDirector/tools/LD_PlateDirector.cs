using System;
using System.Collections.Generic;
using CanTake;
using Hazel;

namespace LuckyExporter
{
	/// <summary>
	/// Plays LD_PlateWaypoints one after another, like RobotDirector with several robots. Each child of Task
	/// List, in name order, is a waypoint naming the robot it moves. Each child of Robots is a robot the
	/// director runs; it tracks which of them a waypoint is controlling (IsControlled, ControlledBy). A robot
	/// with no waypoint running holds its last target.
	///
	/// Arm motion uses only the RobotController's graph inputs and reads nothing back from the robot, so it
	/// carries over to a real one. Rest is an ordinary IK pose. Waypoints move the hand in straight lines, also
	/// out of rest, unless they turn on Solve Rotation or the Elbow Pole. Those and moves into rest use the
	/// graph's Start, which eases the joints there from any pose: it swings the hand in an arc, so it can pass
	/// the end and come back. To approach something in a straight line, turn Solve Rotation on at an earlier
	/// waypoint. The graph has no gripper input, so
	/// the director only says how far open each gripper should be (GripperOpening): something else applies that,
	/// in simulation an LD_PlateSimBridge.
	/// </summary>
	public class LD_PlateDirector : Entity
	{
		[Tooltip("An entity whose children are the robots, each with a RobotController.")]
		public Entity? Robots;

		[DisplayName("Task List")]
		[Tooltip("An entity whose children are the LD_PlateWaypoints, played in name order: 1_Reach, 2_Grip and so on (2 comes before 10).")]
		public Entity? TaskList;

		[ToggleButton("Loop", "Once")]
		[Tooltip("Start again from the first waypoint after the last one finishes.")]
		public bool Loop = true;

		[ToggleButton("Tracking", "Seeded")]
		[Tooltip("Tracking starts each IK solve from the current pose, which keeps the elbow from flipping between moves.")]
		public bool TrackingMode = true;

		[ToggleButton("High Accuracy", "Normal")]
		[Tooltip("Spend more solver iterations per update for a tighter fit to each target.")]
		public bool HighAccuracy = true;

		[DisplayName("Close Strength")]
		[Slider(0.0f, 1.0f)]
		[Tooltip("How hard the grippers close, from 0 to 1. It sets how far past the plate the fingers are commanded, and the grip force follows that, so too low a value may not reach the plate at all.")]
		public float CloseStrength = 1f;

		[DisplayName("Rest Radius")]
		[Units("m")]
		[Slider(0.28f, 0.45f)]
		[Tooltip("How far the resting hand sits from its robot's base axis, in the direction Rest Turn sets. Much closer and the wrist nears that axis, where the IK can flip.")]
		public float RestRadius = 0.282f;

		[DisplayName("Rest Turn")]
		[Units("deg")]
		[Slider(0.0f, 45.0f)]
		[Tooltip("How far the resting hand turns from straight ahead towards the middle of the robots. More keeps the arm's first move shorter, less keeps the hands further from what's on the desk.")]
		public float RestTurn = 6.938f;

		[DisplayName("Rest Height")]
		[Units("m")]
		[Slider(0.15f, 0.45f)]
		[Tooltip("How high above its robot's base the resting hand sits.")]
		public float RestHeight = 0.24f;

		[DisplayName("Rest Droop")]
		[Units("deg")]
		[Slider(0.0f, 60.0f)]
		[Tooltip("How far the resting hand's fingers tip down from level. Too much and the wrist straightens, then bends the other way, and moves out of rest make it flip.")]
		public float RestDroop = 10.0f;

		[DisplayName("Rest Offset")]
		[Units("m")]
		[Tooltip("Moves every resting hand from where the settings above put it, relative to each robot so all of them move alike: X further ahead, Y up, Z sideways towards the middle of the robots. It's read on Play.")]
		public Vector3 RestOffset = Vector3.Zero;

		[DisplayName("Rest Towards Work")]
		[Tooltip("Rest facing the middle of all the waypoints instead of the middle of the robots. Turn on when the robots are all on one side, such as two arms side by side, where the middle of the robots is beside them rather than in front. It's read on Play.")]
		public bool RestTowardsWork = false;

		[DisplayName("Startup Time")]
		[Units("s")]
		[Slider(0.5f, 10.0f)]
		[Tooltip("How long the robots take to ease to rest on Play, before the first waypoint.")]
		public float StartupTime = 1f;

		[DisplayName("Watch While Resting")]
		[Tooltip("A resting robot turns to watch whatever another robot is carrying, the nearest if there are two, and turns back when nothing is carried. The whole arm turns about its base and tips about its shoulder, leaving the wrist alone, as moving the wrist at rest makes it spin. It stops as soon as its next waypoint moves it. Check nothing is in the way of the resting hands' sweep.")]
		public bool WatchWhileResting = true;

		[DisplayName("Watch Limit")]
		[Units("deg")]
		[Slider(0.0f, 90.0f)]
		[Tooltip("How far a watching arm can turn about its base from its rest pose. It tips about its shoulder by up to 25 degrees up and 20 down.")]
		public float WatchLimit = 45.0f;

		[DisplayName("Watch Turn Time")]
		[Units("s")]
		[Slider(0.05f, 2.0f)]
		[Tooltip("How quickly a watching hand follows. Less is snappier, more is lazier.")]
		public float WatchTurnTime = 0.3f;

		[DisplayName("Hide Waypoints")]
		[Tooltip("While playing, hide every waypoint in the Task List and its Elbow Pole sphere. They show again when you stop.")]
		public bool HideWaypoints = true;

		[DisplayName("Currently Controlled")]
		[Tooltip("While playing: the robot a waypoint is controlling, and which waypoint.")]
		public string CurrentlyControlled = "";

		private const float ShoulderHeight = 0.123f;
		private const float PoleReach = 0.5f; // how far along its direction from the arm's middle the pole sits, so the pull depends only on Pole Weight
		private const float GraspToWrist = 0.226f;
		private const float WristNearAxis = 0.06f; // closer than this to its base axis, the IK's solution can flip
		private const float RadPerDeg = MathF.PI / 180.0f;
		private const float DegPerRad = 180.0f / MathF.PI;
		private const float CarriedAhead = 0.08f; // how far along the fingers the middle of something held tends to be
		private const float WatchMaxSpeed = 45.0f * RadPerDeg; // radians per second
		private const float WatchMaxAccel = 90.0f * RadPerDeg; // radians per second squared, so the arm never jerks into or out of a turn
		private const float WatchNearBase = 0.25f; // closer than this across to the base, the direction to what it watches swings fast, so the watch fades out
		private const float HeldOpening = 0.5f; // a gripper closed further than this counts as carrying
		private const float WatchMaxUp = 25.0f * RadPerDeg; // the shoulder's end stop is about 42 degrees up from rest
		private const float WatchMaxDown = 20.0f * RadPerDeg; // keeps the hand about 100 mm above the table

		private sealed class RobotState
		{
			public Entity Entity = null!;
			public RobotControllerComponent Controller = null!;
			public Vector3 Target;
			public Quaternion TargetRotation;
			public bool SolveRotation = true;
			public Vector3 Shoulder;
			public Vector3 PoleDirection;
			public float PoleWeight;
			public Vector3 RestPosition;
			public Quaternion RestRotation;
			public bool AtRest;
			public LD_PlateWaypoint? Waypoint; // the one controlling this robot, if any
			public Vector3 Velocity; // the hand's speed carried out of a Continuous waypoint
			public Vector3 AngularVelocity;
			public float CarriedTime; // how far past the last Continuous waypoint this update's time already is
			public bool Carrying;
			public int FlowsInto = -1; // the waypoint to start as soon as the current one ends
			public float LookYaw; // radians about the base's vertical axis, added to the target while it watches
			public float LookPitch; // radians the arm tips down about its shoulder, added to the target while it watches
			public float LookYawSpeed; // radians per second, so a watch eases in and out instead of starting at full speed
			public float LookPitchSpeed;
			public Vector3 LookPoint; // what it watches, smoothed, so switching between carried things glides
			public bool HasLookPoint;
			public float Opening = 1.0f; // 0 closed .. 1 open
			public float OpeningFrom = 1.0f;
			public float OpeningTo = 1.0f;
			public float OpeningTime;
			public float OpeningElapsed;
		}

		private readonly List<RobotState> m_Robots = new();
		private readonly List<LD_PlateWaypoint> m_Tasks = new();
		private readonly List<RobotState> m_TaskRobots = new();
		private IEnumerator<int>? m_Script;
		private RobotState? m_Active;
		private float m_Delta;
		private int m_FramesSeen;

		// The editor's prefab preview creates scripts on copies outside the running scene, where any
		// engine call crashes the editor. So setup waits for the first update, and a copy does nothing
		private bool m_Started;
		private bool m_Live;

		/// <summary>True while one of this director's waypoints is controlling the robot.</summary>
		public bool IsControlled(Entity robot) => ControlledBy(robot) is not null;

		/// <summary>The waypoint controlling the robot right now, or null.</summary>
		public LD_PlateWaypoint? ControlledBy(Entity robot) => Find(robot)?.Waypoint;

		/// <summary>The robot the latest waypoint to start controls, or null before the first. Others may be running too.</summary>
		public Entity? ActiveRobot => m_Active?.Entity;

		/// <summary>The latest waypoint to start if it's still running, or null.</summary>
		public LD_PlateWaypoint? ActiveWaypoint => m_Active?.Waypoint;

		/// <summary>How far open the robot's gripper should be right now, from 0 fully closed to 1 fully open.</summary>
		public float GripperOpening(Entity robot) => Find(robot)?.Opening ?? 1.0f;

		/// <summary>The hand position the director is asking the robot for right now.</summary>
		public Vector3 CommandedPosition(Entity robot) => Find(robot)?.Target ?? Vector3.Zero;

		private RobotState? Find(Entity robot) => m_Robots.Find(r => r.Entity.Id() == robot.Id());

		protected override void OnUpdate(float deltaTime)
		{
			if (!m_Started)
			{
				m_Started = true;
				m_Live = this; // Entity converts to bool through Scene.IsEntityValid
				if (m_Live)
					m_Live = Setup();
			}
			if (!m_Live)
				return;

			// Let the robots' own first-update setup finish, as it also writes these inputs
			if (++m_FramesSeen < 3)
				return;
			m_Delta = deltaTime;
			m_Script ??= Play().GetEnumerator();
			m_Script.MoveNext();
			foreach (RobotState robot in m_Robots)
			{
				UpdateGripper(robot, deltaTime);
				UpdateWatch(robot, deltaTime);
				WriteTarget(robot);
			}
		}

		private bool Setup()
		{
			if (Robots is null || TaskList is null)
			{
				Log.Warn($"[LD_PlateDirector] {Tag}: set Robots and Task List to entities whose children are the robots and the LD_PlateWaypoints.");
				return false;
			}
			LD_Waypoint.PikaHand = true;

			Entity[] robots = Robots.Children;
			Array.Sort(robots, (a, b) => RobotDirector.NaturalCompare(a.Tag, b.Tag));
			foreach (Entity entity in robots)
			{
				if (entity.GetComponent<RobotControllerComponent>() is RobotControllerComponent controller && RobotInputs.IsSingleArm(entity))
					m_Robots.Add(new RobotState { Entity = entity, Controller = controller });
				else
					Log.Warn($"[LD_PlateDirector] {entity.Tag} isn't a single-arm robot with a RobotController, so it's skipped.");
			}

			Entity[] entries = TaskList.Children;
			Array.Sort(entries, (a, b) => RobotDirector.NaturalCompare(a.Tag, b.Tag));
			if (HideWaypoints)
				foreach (Entity entry in entries)
					HideMeshes(entry);
			for (int i = 0; i < entries.Length; i++)
			{
				if (i > 0 && entries[i].Tag == entries[i - 1].Tag)
					Log.Warn($"[LD_PlateDirector] Two waypoints are both named '{entries[i].Tag}', so their order is arbitrary.");
				LD_PlateWaypoint? waypoint = entries[i].As<LD_PlateWaypoint>();
				RobotState? robot = waypoint?.Robot is Entity target ? Find(target) : null;
				if (waypoint is null)
					Log.Warn($"[LD_PlateDirector] {entries[i].Tag} has no LD_PlateWaypoint script, so it's skipped.");
				else if (robot is null)
					Log.Warn($"[LD_PlateDirector] {entries[i].Tag}: its Robot isn't set to a child of {Robots.Tag}, so it's skipped.");
				else
				{
					m_Tasks.Add(waypoint);
					m_TaskRobots.Add(robot);
					WarnIfNearBaseAxis(waypoint, robot);
				}
			}
			if (m_Tasks.Count == 0)
				Log.Warn($"[LD_PlateDirector] No waypoints to play: parent LD_PlateWaypoints under {TaskList.Tag}.");
			SetRestPoses();
			return m_Robots.Count > 0;
		}

		private static void HideMeshes(Entity entity)
		{
			if (entity.GetComponent<StaticMeshComponent>() is StaticMeshComponent mesh)
				mesh.Visible = false;
			foreach (Entity child in entity.Children)
				HideMeshes(child);
		}

		// With the wrist on the base axis the first and fourth joints do the same thing, so the arm swings about
		private static void WarnIfNearBaseAxis(LD_PlateWaypoint waypoint, RobotState robot)
		{
			float distance = WristAxisDistance(robot, waypoint.HandPosition(), waypoint.HandRotation());
			if (distance < WristNearAxis)
				Log.Warn($"[LD_PlateDirector] {waypoint.Tag}: {robot.Entity.Tag}'s wrist is only {distance * 1000.0f:0} mm from its base axis here, so the arm is likely to glitch. Move the waypoint further from the base.");
		}

		private static float WristAxisDistance(RobotState robot, Vector3 handPosition, Quaternion handRotation)
		{
			Vector3 fingers = handRotation * new Vector3(1.0f, 0.0f, 0.0f);
			Vector3 wrist = handPosition - fingers * GraspToWrist - robot.Entity.Transform.WorldTranslation;
			return MathF.Sqrt(wrist.X * wrist.X + wrist.Z * wrist.Z);
		}

		// Each robot rests facing ahead, along its X axis on the side the other robots are (or the waypoints, with Rest Towards Work), turned towards them by up to Rest Turn, its hand tipped down a little, then moved by Rest Offset
		private void SetRestPoses()
		{
			Vector3 middle = Vector3.Zero;
			if (RestTowardsWork && m_Tasks.Count > 0)
			{
				foreach (LD_PlateWaypoint waypoint in m_Tasks)
					middle += waypoint.HandPosition();
				middle *= 1.0f / m_Tasks.Count;
			}
			else
			{
				foreach (RobotState robot in m_Robots)
					middle += robot.Entity.Transform.WorldTranslation;
				middle *= 1.0f / MathF.Max(m_Robots.Count, 1);
			}

			foreach (RobotState robot in m_Robots)
			{
				Vector3 origin = robot.Entity.Transform.WorldTranslation;
				robot.Shoulder = origin + new Vector3(0.0f, ShoulderHeight, 0.0f);
				Vector3 ahead = robot.Entity.Transform.WorldRotationQuat * new Vector3(1.0f, 0.0f, 0.0f);
				Vector3 facing = new Vector3(ahead.X, 0.0f, ahead.Z).Normalized();
				Vector3 toMiddle = middle - origin;
				if (facing.X * toMiddle.X + facing.Z * toMiddle.Z < 0.0f)
					facing *= -1.0f;
				float aheadYaw = MathF.Atan2(facing.Z, facing.X);
				float off = MathF.Atan2(toMiddle.Z, toMiddle.X) - aheadYaw;
				off = MathF.Atan2(MathF.Sin(off), MathF.Cos(off));
				float turn = RestTurn * RadPerDeg;
				float yaw = aheadYaw + Math.Clamp(off, -turn, turn);
				facing = new Vector3(MathF.Cos(yaw), 0.0f, MathF.Sin(yaw));
				float droop = RestDroop * RadPerDeg;
				Vector3 side = new Vector3(-facing.Z, 0.0f, facing.X);
				if (side.X * toMiddle.X + side.Z * toMiddle.Z < 0.0f)
					side *= -1.0f;
				robot.RestPosition = origin + facing * (RestRadius + RestOffset.X) + new Vector3(0.0f, RestHeight + RestOffset.Y, 0.0f) + side * RestOffset.Z;
				robot.RestRotation = HandRotation(facing * MathF.Cos(droop) + new Vector3(0.0f, -MathF.Sin(droop), 0.0f));
				float restDistance = WristAxisDistance(robot, robot.RestPosition, robot.RestRotation);
				if (restDistance < WristNearAxis)
					Log.Warn($"[LD_PlateDirector] {robot.Entity.Tag}'s resting wrist is only {restDistance * 1000.0f:0} mm from its base axis, so the arm is likely to glitch. Change Rest Radius or Rest Offset.");
				robot.Target = robot.RestPosition;
				robot.TargetRotation = robot.RestRotation;
			}
		}

		// ---- The sequence. Each yield waits for the next update ----

		private IEnumerable<int> Play()
		{
			foreach (RobotState robot in m_Robots)
			{
				robot.Controller.SetInputBool(RobotInput.LeftArm_High_Accuracy.ForArm(0, true), HighAccuracy);
				StartEase(robot, robot.RestPosition, robot.RestRotation, StartupTime);
			}
			foreach (int _ in Wait(StartupTime))
				yield return 0;
			foreach (RobotState robot in m_Robots)
			{
				EndEase(robot);
				robot.AtRest = true;
			}

			int count = m_Tasks.Count;
			var running = new List<IEnumerator<int>>();
			var ends = new float[count];
			var startedEarly = new bool[count]; // flowed into ahead of its place in the list
			float clock = 0.0f;
			int next = 0;

			void Start(int index)
			{
				RobotState robot = m_TaskRobots[index];
				int flowsInto = NextInMotion(index, robot);
				robot.FlowsInto = flowsInto;
				LD_PlateWaypoint? flowWaypoint = flowsInto >= 0 ? m_Tasks[flowsInto] : null;
				ends[index] = clock + TaskTime(m_Tasks[index], robot, flowWaypoint);
				IEnumerator<int> task = RunTask(index, flowWaypoint).GetEnumerator();
				if (task.MoveNext())
					running.Add(task);
			}

			while (true)
			{
				clock += m_Delta;
				running.RemoveAll(task => !task.MoveNext());

				// A Continuous waypoint carries straight on, whatever the other robots are doing
				foreach (RobotState robot in m_Robots)
				{
					if (robot.Waypoint is null && robot.FlowsInto >= 0)
					{
						int index = robot.FlowsInto;
						robot.FlowsInto = -1;
						startedEarly[index] = true;
						Start(index);
					}
				}

				for (int steps = 0; next < count && steps < count; steps++)
				{
					if (!startedEarly[next])
					{
						if (!CanStart(next, running.Count, ends[(next + count - 1) % count] - clock))
							break;
						Start(next);
					}
					startedEarly[next] = false;
					next = next + 1 == count && Loop ? 0 : next + 1;
				}
				CurrentlyControlled = string.Join(", ", m_Robots.FindAll(r => r.Waypoint is not null).ConvertAll(r => $"{r.Entity.Tag}: {r.Waypoint!.Tag}"));
				if (next >= count && running.Count == 0)
					break;
				yield return 0;
			}
		}

		// A waypoint waits for everything before it, unless it overlaps the last one
		private bool CanStart(int index, int runningCount, float lastTimeLeft)
		{
			if (m_TaskRobots[index].Waypoint is not null)
				return false;
			return runningCount == 0 || m_Tasks[index].OverlapFromLast > 0.0f && lastTimeLeft <= m_Tasks[index].OverlapFromLast;
		}

		// How long RunTask will take
		private static float TaskTime(LD_PlateWaypoint waypoint, RobotState robot, LD_PlateWaypoint? flowsInto)
		{
			float time = waypoint.WaitStart + MathF.Max(waypoint.Duration, 0.01f);
			if (waypoint.Rest == LD_PlateWaypoint.RestOnStart && !robot.AtRest)
				time += waypoint.RestTime;
			if (flowsInto is not null)
				return time;
			if (waypoint.GripsOnStart)
				time += MathF.Max(waypoint.GripperTime - waypoint.Duration, 0.0f);
			if (waypoint.GripsOnEnd)
				time += waypoint.GripperTime;
			time += waypoint.WaitEnd;
			if (waypoint.Rest == LD_PlateWaypoint.RestOnEnd)
				time += waypoint.RestTime;
			return time;
		}

		private IEnumerable<int> RunTask(int index, LD_PlateWaypoint? next)
		{
			LD_PlateWaypoint waypoint = m_Tasks[index];
			RobotState robot = m_TaskRobots[index];
			robot.Waypoint = waypoint;
			m_Active = robot;

			foreach (int _ in Wait(waypoint.WaitStart))
				yield return 0;

			bool fromRest = waypoint.Rest == LD_PlateWaypoint.RestOnStart;
			if (fromRest && !robot.AtRest)
				foreach (int _ in GoToRest(robot, waypoint.RestTime))
					yield return 0;

			if (waypoint.GripsOnStart)
				SetGripper(robot, GripperGoal(waypoint), waypoint.GripperTime);

			robot.AtRest = false;
			// Nothing here knows the hand's rotation while it's free, so going back to solving it could snap. The graph's ease covers that.
			// The solver jumps when its pole pull starts or stops, so that's covered too.
			bool poleChanges = waypoint.PoleDirection().HasValue != robot.PoleWeight > 0.0f;
			IEnumerable<int> move = poleChanges || waypoint.SolveRotation && !robot.SolveRotation ? EaseTo(robot, waypoint) : MoveTo(robot, waypoint, next);
			foreach (int _ in move)
				yield return 0;

			if (next is null)
			{
				if (waypoint.GripsOnStart)
					foreach (int _ in Wait(waypoint.GripperTime - waypoint.Duration))
						yield return 0;
				if (waypoint.GripsOnEnd)
				{
					SetGripper(robot, GripperGoal(waypoint), waypoint.GripperTime);
					foreach (int _ in Wait(waypoint.GripperTime))
						yield return 0;
				}
				foreach (int _ in Wait(waypoint.WaitEnd))
					yield return 0;

				if (waypoint.Rest == LD_PlateWaypoint.RestOnEnd)
					foreach (int _ in GoToRest(robot, waypoint.RestTime))
						yield return 0;
			}

			robot.Waypoint = null;
		}

		// The index of this robot's next waypoint if a Continuous one flows into it, or -1 if the robot stops here
		private int NextInMotion(int index, RobotState robot)
		{
			LD_PlateWaypoint waypoint = m_Tasks[index];
			if (!waypoint.Continuous)
				return -1;
			int following = -1;
			for (int step = 1; step < m_Tasks.Count && following < 0; step++)
			{
				int candidate = index + step;
				if (candidate >= m_Tasks.Count && !Loop)
					break;
				candidate %= m_Tasks.Count;
				if (m_TaskRobots[candidate] == robot)
					following = candidate;
			}
			string? stop = following < 0 ? "it's the robot's last one"
				: waypoint.GripsOnEnd || waypoint.WaitEnd > 0.0f || waypoint.Rest == LD_PlateWaypoint.RestOnEnd ? "it has a gripper move, wait or rest at its end"
				: waypoint.SolveRotation != m_Tasks[following].SolveRotation ? "its next waypoint changes Solve Rotation"
				: waypoint.PoleDirection().HasValue != m_Tasks[following].PoleDirection().HasValue ? "its next waypoint turns the Elbow Pole on or off"
				: m_Tasks[following].WaitStart > 0.0f || m_Tasks[following].Rest == LD_PlateWaypoint.RestOnStart ? "its next waypoint starts with a wait or rest"
				: null;
			if (stop is null)
				return following;
			Log.Warn($"[LD_PlateDirector] {waypoint.Tag} is Continuous, but the robot stops there as {stop}.");
			return -1;
		}

		private float GripperGoal(LD_PlateWaypoint waypoint) => waypoint.GripperOpening > 0.0f ? 1.0f : 1.0f - Math.Clamp(CloseStrength, 0.0f, 1.0f);

		// Eases the hand along a straight line, turning as it goes. A quintic blend: with no speed at either
		// end it's a plain ease, and given speeds at the ends it joins the moves either side without a jolt.
		private IEnumerable<int> MoveTo(RobotState robot, LD_PlateWaypoint waypoint, LD_PlateWaypoint? next)
		{
			Vector3 start = robot.Target;
			Quaternion startRotation = robot.TargetRotation;
			Vector3 position = waypoint.HandPosition();
			Quaternion rotation = waypoint.HandRotation();
			float time = MathF.Max(waypoint.Duration, 0.01f);

			Vector3 startVelocity = robot.Carrying ? robot.Velocity : Vector3.Zero;
			Vector3 startAngular = robot.Carrying ? robot.AngularVelocity : Vector3.Zero;
			Vector3 turn = RotationBetween(startRotation, rotation);
			Vector3 endVelocity = Vector3.Zero;
			Vector3 endAngular = Vector3.Zero;
			if (next is not null)
			{
				// The hand's speed through this waypoint: the average of the moves either side
				float nextTime = MathF.Max(next.Duration, 0.01f);
				endVelocity = ((position - start) * (1.0f / time) + (next.HandPosition() - position) * (1.0f / nextTime)) * 0.5f;
				endAngular = (turn * (1.0f / time) + RotationBetween(rotation, next.HandRotation()) * (1.0f / nextTime)) * 0.5f;
			}

			// The elbow's pull turns from one waypoint's direction and weight to the next over the move
			float startPoleWeight = robot.PoleWeight;
			Vector3 startPole = robot.PoleDirection;
			Vector3 endPole = waypoint.PoleDirection() ?? startPole;
			float endPoleWeight = waypoint.PoleDirection().HasValue ? Math.Clamp(waypoint.PoleWeight, 0.0f, 1.0f) : 0.0f;
			robot.SolveRotation = waypoint.SolveRotation;

			float elapsed = robot.Carrying ? robot.CarriedTime : m_Delta;
			robot.Carrying = false;
			for (; ; elapsed += m_Delta)
			{
				float u = Math.Clamp(elapsed / time, 0.0f, 1.0f);
				float u3 = u * u * u, u4 = u3 * u, u5 = u4 * u;
				float toEnd = 10.0f * u3 - 15.0f * u4 + 6.0f * u5;
				float withStart = (u - 6.0f * u3 + 8.0f * u4 - 3.0f * u5) * time;
				float withEnd = (-4.0f * u3 + 7.0f * u4 - 3.0f * u5) * time;
				robot.Target = start + (position - start) * toEnd + startVelocity * withStart + endVelocity * withEnd;
				Vector3 turned = turn * toEnd + startAngular * withStart + endAngular * withEnd;
				robot.TargetRotation = (FromRotationVector(turned) * startRotation).Normalized();
				Vector3 poleBetween = startPole + (endPole - startPole) * toEnd;
				robot.PoleDirection = poleBetween.Length() > 0.001f ? poleBetween.Normalized() : endPole;
				robot.PoleWeight = startPoleWeight + (endPoleWeight - startPoleWeight) * toEnd;
				if (u >= 1.0f)
					break;
				yield return 0;
			}

			robot.Target = position;
			robot.TargetRotation = rotation;
			robot.PoleDirection = endPole;
			robot.PoleWeight = endPoleWeight;
			if (next is not null)
			{
				robot.Velocity = endVelocity;
				robot.AngularVelocity = endAngular;
				robot.CarriedTime = elapsed - time;
				robot.Carrying = true;
			}
		}

		// The world-frame rotation taking a to b, as axis times angle, the short way round
		private static Vector3 RotationBetween(Quaternion a, Quaternion b)
		{
			Quaternion delta = (b * a.Conjugate).Normalized();
			Vector3 axis = new Vector3(delta.X, delta.Y, delta.Z);
			float w = delta.W;
			if (w < 0.0f)
			{
				axis *= -1.0f;
				w = -w;
			}
			float sine = axis.Length();
			return sine < 1e-6f ? axis * 2.0f : axis * (2.0f * MathF.Atan2(sine, w) / sine);
		}

		private static Quaternion FromRotationVector(Vector3 v)
		{
			float angle = v.Length();
			if (angle < 1e-6f)
				return new Quaternion(v.X * 0.5f, v.Y * 0.5f, v.Z * 0.5f, 1.0f).Normalized();
			Vector3 axis = v * (MathF.Sin(angle * 0.5f) / angle);
			return new Quaternion(axis.X, axis.Y, axis.Z, MathF.Cos(angle * 0.5f));
		}

		private IEnumerable<int> GoToRest(RobotState robot, float time)
		{
			StartEase(robot, robot.RestPosition, robot.RestRotation, time);
			foreach (int _ in Wait(time))
				yield return 0;
			EndEase(robot);
			robot.AtRest = true;
		}

		private IEnumerable<int> EaseTo(RobotState robot, LD_PlateWaypoint waypoint)
		{
			StartEase(robot, waypoint.HandPosition(), waypoint.HandRotation(), waypoint.Duration);
			robot.SolveRotation = waypoint.SolveRotation;
			if (waypoint.PoleDirection() is Vector3 pole)
			{
				robot.PoleDirection = pole;
				robot.PoleWeight = Math.Clamp(waypoint.PoleWeight, 0.0f, 1.0f);
			}
			foreach (int _ in Wait(waypoint.Duration))
				yield return 0;
			EndEase(robot);
		}

		// The graph eases the joints from where they are to the IK's solution for this target. Seeded, as
		// the tracking solver would chase the joints as they move.
		private static void StartEase(RobotState robot, Vector3 position, Quaternion rotation, float time)
		{
			robot.Target = position;
			robot.TargetRotation = rotation;
			robot.SolveRotation = true;
			robot.PoleWeight = 0.0f; // the graph's ease covers the change
			robot.LookYaw = 0.0f;
			robot.LookPitch = 0.0f;
			robot.LookYawSpeed = 0.0f;
			robot.LookPitchSpeed = 0.0f;
			robot.HasLookPoint = false;
			robot.Controller.SetInputBool(RobotInput.LeftArm_Tracking_Mode.ForArm(0, true), false);
			WriteTarget(robot);
			robot.Controller.SetInputFloat(RobotInput.Duration.ID(), MathF.Max(time, 0.01f));
			robot.Controller.SetInputTrigger(RobotInput.Start.ID());
		}

		private void EndEase(RobotState robot) => robot.Controller.SetInputBool(RobotInput.LeftArm_Tracking_Mode.ForArm(0, true), TrackingMode);

		private IEnumerable<int> Wait(float time)
		{
			for (float t = 0.0f; t < time; t += m_Delta)
				yield return 0;
		}

		// ---- Watching ----

		// Only what the director commands, so a real robot watches the same way. The turn is added when writing the target, so the
		// target itself never changes, and ending the watch leaves the robot exactly where its waypoints expect. Turning only the hand
		// can't work at rest, as the wrist is near its dead spot, and tipping it down goes through that spot, spinning it at 15 rad/s.
		// Turning about the base and tipping about the shoulder move the arm as one rigid piece, so only joints 1 and 2 move.
		private void UpdateWatch(RobotState robot, float deltaTime)
		{
			float wantedYaw = 0.0f;
			float wantedPitch = 0.0f;
			float turnTime = MathF.Max(WatchTurnTime, 0.05f);
			if (WatchWhileResting && robot.AtRest && FindCarried(robot) is Vector3 carried)
			{
				// Glide the watched point, so a switch to another carried thing, or a jump in one, doesn't snap the goal
				if (robot.HasLookPoint)
					robot.LookPoint += (carried - robot.LookPoint) * (1.0f - MathF.Exp(-deltaTime / turnTime));
				else
					robot.LookPoint = carried;
				robot.HasLookPoint = true;
				Vector3 point = robot.LookPoint;

				float limit = WatchLimit * RadPerDeg;
				Vector3 origin = robot.Entity.Transform.WorldTranslation;
				Vector3 fingers = robot.TargetRotation * new Vector3(1.0f, 0.0f, 0.0f);
				// The hand rests on a line out from the base, so facing something from the base also faces it from the hand. About +Y, angles run from +X away from +Z.
				float off = MathF.Atan2(fingers.Z, fingers.X) - MathF.Atan2(point.Z - origin.Z, point.X - origin.X);
				wantedYaw = Math.Clamp(MathF.Atan2(MathF.Sin(off), MathF.Cos(off)), -limit, limit);
				// Worked out afresh each update from the hand turned but untipped, rather than from the tip it has now,
				// which fed back on itself and chattered when something passed close. Tipping down lowers the hand, which asks for less tip.
				float fingersUp = MathF.Asin(Math.Clamp(fingers.Y, -1.0f, 1.0f));
				float pitch = 0.0f;
				for (int i = 0; i < 4; i++)
				{
					WatchingPose(robot, wantedYaw, pitch, out Vector3 hand, out _);
					float across = new Vector3(point.X - hand.X, 0.0f, point.Z - hand.Z).Length();
					float down = MathF.Atan2(hand.Y - point.Y, MathF.Max(across, 0.05f));
					pitch = Math.Clamp(0.5f * (pitch + down + fingersUp), -MathF.Min(limit, WatchMaxUp), MathF.Min(limit, WatchMaxDown));
				}
				wantedPitch = pitch;
				// Fade out as it nears the base, where the direction to it swings round quickly
				float fromBase = new Vector3(point.X - origin.X, 0.0f, point.Z - origin.Z).Length();
				float fade = Math.Clamp((fromBase - WatchNearBase * 0.5f) / (WatchNearBase * 0.5f), 0.0f, 1.0f);
				fade = fade * fade * (3.0f - 2.0f * fade);
				wantedYaw *= fade;
				wantedPitch *= fade;
			}
			else
				robot.HasLookPoint = false;
			robot.LookYaw = EaseAngle(robot.LookYaw, wantedYaw, ref robot.LookYawSpeed, turnTime, deltaTime);
			robot.LookPitch = EaseAngle(robot.LookPitch, wantedPitch, ref robot.LookPitchSpeed, turnTime, deltaTime);
		}

		// A critically damped spring with capped speed and acceleration, so a watch starts and stops gently
		private static float EaseAngle(float current, float wanted, ref float speed, float turnTime, float deltaTime)
		{
			float omega = 2.0f / turnTime;
			int steps = Math.Clamp((int)MathF.Ceiling(deltaTime * 240.0f), 1, 32);
			float dt = deltaTime / steps;
			for (int i = 0; i < steps; i++)
			{
				float accel = omega * omega * (wanted - current) - 2.0f * omega * speed;
				speed += Math.Clamp(accel, -WatchMaxAccel, WatchMaxAccel) * dt;
				speed = Math.Clamp(speed, -WatchMaxSpeed, WatchMaxSpeed);
				current += speed * dt;
			}
			return current;
		}

		// The middle of what the nearest other robot is carrying, or null
		private Vector3? FindCarried(RobotState watcher)
		{
			Vector3? nearest = null;
			float nearestDistance = float.MaxValue;
			foreach (RobotState robot in m_Robots)
			{
				if (robot == watcher || robot.Waypoint is null || robot.Opening > HeldOpening)
					continue;
				Vector3 middle = robot.Target + robot.TargetRotation * new Vector3(CarriedAhead, 0.0f, 0.0f);
				float distance = (middle - watcher.Target).Length();
				if (distance < nearestDistance)
				{
					nearest = middle;
					nearestDistance = distance;
				}
			}
			return nearest;
		}

		// ---- Writing to the robots ----

		private static void WriteTarget(RobotState robot)
		{
			robot.Controller.SetInputBool(RobotInput.LeftArm_Solve_Orientation.ForArm(0, true), robot.SolveRotation);
			WatchingPose(robot, robot.LookYaw, robot.LookPitch, out Vector3 position, out Quaternion rotation);
			robot.Controller.SetInputVector3(RobotInput.LeftArm_Target_Position.ForArm(0, true), position);
			robot.Controller.SetInputVector3(RobotInput.LeftArm_Target_Orientation.ForArm(0, true), rotation.EulerAngles * DegPerRad);
			Vector3 armMiddle = (robot.Shoulder + robot.Target) * 0.5f;
			robot.Controller.SetInputVector3(RobotInput.LeftArm_Pole_Target.ForArm(0, true), armMiddle + robot.PoleDirection * PoleReach);
			robot.Controller.SetInputFloat(RobotInput.LeftArm_Pole_Weight.ForArm(0, true), robot.PoleWeight);
		}

		// The target with the watching turn added: the whole arm tips about its shoulder, then turns about its base
		private static void WatchingPose(RobotState robot, float yaw, float pitch, out Vector3 position, out Quaternion rotation)
		{
			position = robot.Target;
			rotation = robot.TargetRotation;
			if (yaw == 0.0f && pitch == 0.0f)
				return;
			Vector3 reach = position - robot.Shoulder;
			// The shoulder's axis is square to the plane the wrist is in, not the hand, and any other axis has to move the wrist
			Vector3 ahead = position - rotation * new Vector3(GraspToWrist, 0.0f, 0.0f) - robot.Shoulder;
			ahead.Y = 0.0f;
			if (ahead.Length() > 0.001f)
			{
				// Up cross ahead, which tips the arm down
				Quaternion tip = FromRotationVector(new Vector3(ahead.Z, 0.0f, -ahead.X).Normalized() * pitch);
				position = robot.Shoulder + tip * reach;
				rotation = tip * rotation;
			}
			Quaternion turn = FromRotationVector(new Vector3(0.0f, yaw, 0.0f));
			position = robot.Shoulder + turn * (position - robot.Shoulder);
			rotation = (turn * rotation).Normalized();
		}

		// The grasp site's frame: X along the fingers, Y the way the jaws open, Z the hand's top, as near up as the fingers allow
		private static Quaternion HandRotation(Vector3 fingers)
		{
			Vector3 x = fingers.Normalized();
			Vector3 z = (new Vector3(0.0f, 1.0f, 0.0f) - x * x.Y).Normalized();
			Vector3 y = new Vector3(z.Y * x.Z - z.Z * x.Y, z.Z * x.X - z.X * x.Z, z.X * x.Y - z.Y * x.X);
			float trace = x.X + y.Y + z.Z;
			float qx, qy, qz, qw;
			if (trace > 0.0f)
			{
				float s = MathF.Sqrt(trace + 1.0f) * 2.0f;
				qw = 0.25f * s; qx = (y.Z - z.Y) / s; qy = (z.X - x.Z) / s; qz = (x.Y - y.X) / s;
			}
			else if (x.X > y.Y && x.X > z.Z)
			{
				float s = MathF.Sqrt(1.0f + x.X - y.Y - z.Z) * 2.0f;
				qw = (y.Z - z.Y) / s; qx = 0.25f * s; qy = (y.X + x.Y) / s; qz = (z.X + x.Z) / s;
			}
			else if (y.Y > z.Z)
			{
				float s = MathF.Sqrt(1.0f + y.Y - x.X - z.Z) * 2.0f;
				qw = (z.X - x.Z) / s; qx = (y.X + x.Y) / s; qy = 0.25f * s; qz = (z.Y + y.Z) / s;
			}
			else
			{
				float s = MathF.Sqrt(1.0f + z.Z - x.X - y.Y) * 2.0f;
				qw = (x.Y - y.X) / s; qx = (z.X + x.Z) / s; qy = (z.Y + y.Z) / s; qz = 0.25f * s;
			}
			return new Quaternion(qx, qy, qz, qw).Normalized();
		}

		private static void SetGripper(RobotState robot, float opening, float time)
		{
			robot.OpeningFrom = robot.Opening;
			robot.OpeningTo = opening;
			robot.OpeningTime = time;
			robot.OpeningElapsed = 0.0f;
		}

		// Eased, as snapping the jaws flicks the plate
		private static void UpdateGripper(RobotState robot, float deltaTime)
		{
			robot.OpeningElapsed += deltaTime;
			float t = robot.OpeningTime <= 0.0f ? 1.0f : Math.Clamp(robot.OpeningElapsed / robot.OpeningTime, 0.0f, 1.0f);
			robot.Opening = robot.OpeningFrom + (robot.OpeningTo - robot.OpeningFrom) * t * t * (3.0f - 2.0f * t);
		}
	}
}
