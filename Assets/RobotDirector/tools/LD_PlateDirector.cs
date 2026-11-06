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

		[DisplayName("Startup Time")]
		[Units("s")]
		[Slider(0.5f, 10.0f)]
		[Tooltip("How long the robots take to ease to rest on Play, before the first waypoint.")]
		public float StartupTime = 1f;

		[DisplayName("Currently Controlled")]
		[Tooltip("While playing: the robot a waypoint is controlling, and which waypoint.")]
		public string CurrentlyControlled = "";

		private const float ShoulderHeight = 0.123f;
		private const float PoleReach = 0.5f; // how far along its direction from the arm's middle the pole sits, so the pull depends only on Pole Weight
		private const float GraspToWrist = 0.226f;
		private const float WristNearAxis = 0.06f; // closer than this to its base axis, the IK's solution can flip
		private const float RadPerDeg = MathF.PI / 180.0f;
		private const float DegPerRad = 180.0f / MathF.PI;

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

		private RobotState? Find(Entity robot) => m_Robots.Find(r => r.Entity.Handle == robot.Handle);

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
			SetRestPoses();

			Entity[] entries = TaskList.Children;
			Array.Sort(entries, (a, b) => RobotDirector.NaturalCompare(a.Tag, b.Tag));
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
			return m_Robots.Count > 0;
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

		// Each robot rests facing ahead, along its X axis on the side the other robots are, turned towards them by up to Rest Turn, its hand tipped down a little, then moved by Rest Offset
		private void SetRestPoses()
		{
			Vector3 middle = Vector3.Zero;
			foreach (RobotState robot in m_Robots)
				middle += robot.Entity.Transform.WorldTranslation;
			middle *= 1.0f / MathF.Max(m_Robots.Count, 1);

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

			var running = new List<IEnumerator<int>>();
			float clock = 0.0f;
			float lastEnd = 0.0f;
			int next = 0;
			while (true)
			{
				clock += m_Delta;
				running.RemoveAll(task => !task.MoveNext());
				for (int started = 0; next < m_Tasks.Count && started < m_Tasks.Count && CanStart(next, running.Count, lastEnd - clock); started++)
				{
					RobotState robot = m_TaskRobots[next];
					LD_PlateWaypoint? flowsInto = NextInMotion(next, robot);
					lastEnd = clock + TaskTime(m_Tasks[next], robot, flowsInto);
					IEnumerator<int> task = RunTask(next, flowsInto).GetEnumerator();
					if (task.MoveNext())
						running.Add(task);
					next = next + 1 == m_Tasks.Count && Loop ? 0 : next + 1;
				}
				CurrentlyControlled = string.Join(", ", m_Robots.FindAll(r => r.Waypoint is not null).ConvertAll(r => $"{r.Entity.Tag}: {r.Waypoint!.Tag}"));
				if (next >= m_Tasks.Count && running.Count == 0)
					break;
				yield return 0;
			}
		}

		// A waypoint waits for everything before it, unless it overlaps the last one, or that one is flowing straight into it.
		private bool CanStart(int index, int runningCount, float lastTimeLeft)
		{
			RobotState robot = m_TaskRobots[index];
			if (robot.Waypoint is not null)
				return false;
			return runningCount == 0 || robot.Carrying || m_Tasks[index].OverlapFromLast > 0.0f && lastTimeLeft <= m_Tasks[index].OverlapFromLast;
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

		// The waypoint a Continuous one flows into, or null if the robot stops here
		private LD_PlateWaypoint? NextInMotion(int index, RobotState robot)
		{
			LD_PlateWaypoint waypoint = m_Tasks[index];
			if (!waypoint.Continuous)
				return null;
			int following = index + 1 < m_Tasks.Count ? index + 1 : Loop ? 0 : -1;
			string? stop = following < 0 ? "it's the last one"
				: waypoint.GripsOnEnd || waypoint.WaitEnd > 0.0f || waypoint.Rest == LD_PlateWaypoint.RestOnEnd ? "it has a gripper move, wait or rest at its end"
				: m_TaskRobots[following] != robot ? "the next waypoint is another robot's"
				: waypoint.SolveRotation != m_Tasks[following].SolveRotation ? "the next waypoint changes Solve Rotation"
				: waypoint.PoleDirection().HasValue != m_Tasks[following].PoleDirection().HasValue ? "the next waypoint turns the Elbow Pole on or off"
				: m_Tasks[following].WaitStart > 0.0f || m_Tasks[following].Rest == LD_PlateWaypoint.RestOnStart ? "the next waypoint starts with a wait or rest"
				: null;
			if (stop is null)
				return m_Tasks[following];
			Log.Warn($"[LD_PlateDirector] {waypoint.Tag} is Continuous, but the robot stops there as {stop}.");
			return null;
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

		// ---- Writing to the robots ----

		private static void WriteTarget(RobotState robot)
		{
			robot.Controller.SetInputBool(RobotInput.LeftArm_Solve_Orientation.ForArm(0, true), robot.SolveRotation);
			robot.Controller.SetInputVector3(RobotInput.LeftArm_Target_Position.ForArm(0, true), robot.Target);
			robot.Controller.SetInputVector3(RobotInput.LeftArm_Target_Orientation.ForArm(0, true), robot.TargetRotation.EulerAngles * DegPerRad);
			Vector3 armMiddle = (robot.Shoulder + robot.Target) * 0.5f;
			robot.Controller.SetInputVector3(RobotInput.LeftArm_Pole_Target.ForArm(0, true), armMiddle + robot.PoleDirection * PoleReach);
			robot.Controller.SetInputFloat(RobotInput.LeftArm_Pole_Weight.ForArm(0, true), robot.PoleWeight);
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
