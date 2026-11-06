using System;
using System.Collections.Generic;
using Hazel;

namespace LuckyExporter
{
	/// <summary>
	/// Simulation only: everything LD_PlateDirector needs that the RobotController can't do. Put it on its own
	/// entity, and leave it out on a real robot, which needs its own way to apply GripperOpening.
	///
	/// It applies each robot's gripper opening to its MuJoCo actuator, as the motion graph has no gripper
	/// input. It also keeps each named object together: every Piper has its own MuJoCo world with its own copy of
	/// every free body, and only that world's fingers touch that copy, so an object is simulated in one robot's
	/// world and the other worlds' copies mirror that pose. That's the robot gripping it, or else the nearest robot
	/// a waypoint is controlling. A free body left off the list has unsynced copies, and the fingers appear to pass
	/// through the one on screen. Names must be unique, as an object is found by its name.
	///
	/// The director reads nothing from here. This also reads the hand and joints, to warn about missed
	/// waypoints and for Log Motion. No Entity fields, as those crash this editor build when added unassigned.
	/// </summary>
	public class LD_PlateSimBridge : Entity
	{
		[DisplayName("Director Name")]
		[Tooltip("Exact name of the entity with the LD_PlateDirector.")]
		public string DirectorName = "LD_PlateDirector";

		[DisplayName("Plate Names")]
		[Tooltip("Exact names of the objects the arms can pick up, each a free body, separated by commas: IKEA+365++Plate, paper. Empty for none. Set it before pressing Play, as it's read when Play starts.")]
		public string PlateName = "IKEA+365++Plate";

		[DisplayName("Log Motion")]
		[Tooltip("Logs the controlled robot's target, hand and joints ten times a second, for tuning a sequence.")]
		public bool LogMotion = false;

		private const float LogInterval = 0.1f;
		private const float MissWarning = 0.01f; // metres off target when a waypoint ends
		private const float PlateBodySearch = 0.05f;
		private const float GripReach = 0.3f; // how far an object's origin can be from a hand gripping it
		private const float KeepOwnerBias = 0.02f; // stops an object swapping worlds while two hands are about as near
		private const int ArmJoints = 6;
		private const string LeftPad = "pika_left_finger_collision";
		private const string RightPad = "pika_right_finger_collision";

		private sealed class SimRobot
		{
			public Entity Entity = null!;
			public MujocoSceneComponent Mujoco = null!;
			public MujocoActuator? Gripper;
			public double GripperOpen;
			public double GripperClosed;
			public uint GraspSite;
			public uint[] JointAddresses = Array.Empty<uint>();
			public LD_PlateWaypoint? LastWaypoint;
			public Vector3 LastTarget;
		}

		private sealed class SimObject
		{
			public Entity Entity = null!;
			public readonly Dictionary<SimRobot, uint> Bodies = new(); // its copy in each robot's world
			public SimRobot? Owner; // the robot whose world simulates it
		}

		private readonly List<SimRobot> m_Robots = new();
		private readonly List<SimObject> m_Objects = new();
		private LD_PlateDirector? m_Director;
		private float m_LogTime;
		private float m_LogWait;
		private bool m_Started;
		private bool m_Live;

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

			foreach (SimRobot robot in m_Robots)
				if (robot.Gripper is not null)
					robot.Gripper.Value = robot.GripperClosed + (robot.GripperOpen - robot.GripperClosed) * m_Director!.GripperOpening(robot.Entity);
			foreach (SimObject simObject in m_Objects)
				UpdateObject(simObject);
			WarnOnMiss();
			if (LogMotion)
				UpdateMotionLog(deltaTime);
		}

		private bool Setup()
		{
			m_Director = FindEntityByTag(DirectorName)?.As<LD_PlateDirector>();
			if (m_Director?.Robots is null)
			{
				Log.Warn($"[LD_PlateSimBridge] {Tag}: no LD_PlateDirector with its Robots set on an entity named '{DirectorName}'.");
				return false;
			}
			foreach (Entity entity in m_Director.Robots.Children)
				if (entity.GetComponent<MujocoSceneComponent>() is MujocoSceneComponent mujoco)
					m_Robots.Add(AddRobot(entity, mujoco));
			foreach (string name in (PlateName ?? "").Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
				AddObject(name);
			return m_Robots.Count > 0;
		}

		private void AddObject(string name)
		{
			Entity? entity = FindEntityByTag(name);
			if (entity is null)
			{
				Log.Warn($"[LD_PlateSimBridge] No entity named '{name}'.");
				return;
			}
			var simObject = new SimObject { Entity = entity };
			foreach (SimRobot robot in m_Robots)
			{
				if (FindBody(robot.Mujoco, entity, out uint body))
					simObject.Bodies[robot] = body;
				else
					Log.Warn($"[LD_PlateSimBridge] {robot.Entity.Tag}'s physics has no body at {name}. Give it a collider and a free joint.");
			}
			if (simObject.Bodies.Count > 0)
				m_Objects.Add(simObject);
		}

		private SimRobot AddRobot(Entity entity, MujocoSceneComponent mujoco)
		{
			var robot = new SimRobot { Entity = entity, Mujoco = mujoco, GraspSite = mujoco.GetSiteID("grasp_site") };
			if (mujoco.GetActuatorID("gripper") >= 0)
			{
				robot.Gripper = mujoco.GetActuator("gripper");
				robot.Gripper!.IK = false;
				if (mujoco.GetActuatorCtrlRange(mujoco.GetActuatorIndex("gripper"), out float low, out float high) && high > low)
				{
					// The Pika's slide is fully open at the end of its range nearest zero
					bool highIsOpen = MathF.Abs(high) <= MathF.Abs(low);
					robot.GripperOpen = highIsOpen ? high : low;
					robot.GripperClosed = highIsOpen ? low : high;
				}
			}
			var addresses = new uint[ArmJoints];
			for (int i = 0; i < ArmJoints; i++)
				addresses[i] = mujoco.GetJointQposAdr(mujoco.GetJointID($"joint{i + 1}"));
			robot.JointAddresses = addresses;
			return robot;
		}

		// Scene bodies are named from their entity's UUID, which scripts can't read, so an object's is
		// the scene body found at it. Every world compiled its own copy there.
		private static bool FindBody(MujocoSceneComponent mujoco, Entity entity, out uint found)
		{
			found = 0;
			Vector3 target = entity.Transform.WorldTranslation;
			Quaternion mjToHz = mujoco.GetAxisHzToMj().Conjugate;
			Vector3 origin = mujoco.GetSceneTranslation();
			float nearest = PlateBodySearch;
			for (uint body = 1; body < mujoco.ModelNbody; body++)
			{
				if (!mujoco.GetBodyNameByIndex(body).StartsWith("lucky_body_")
					|| !mujoco.GetBodyPoseMj(body, out Vector3 position, out Quaternion _, out Vector3 _, out Vector3 _))
					continue;
				float distance = (mjToHz * position + origin - target).Length();
				if (distance < nearest)
				{
					nearest = distance;
					found = body;
				}
			}
			return found != 0;
		}

		// The new owner's copy, mirrored until now, sets off from the same pose at rest.
		private void UpdateObject(SimObject simObject)
		{
			SimRobot? wanted = WantedOwner(simObject);
			if (wanted is not null && wanted != simObject.Owner)
			{
				wanted.Mujoco.UnweldBody(simObject.Bodies[wanted]);
				simObject.Owner = wanted;
			}
			simObject.Owner ??= m_Robots.Find(r => simObject.Bodies.ContainsKey(r));
			if (simObject.Owner is null)
				return;

			MujocoSceneComponent owner = simObject.Owner.Mujoco;
			if (!owner.GetBodyPoseMj(simObject.Bodies[simObject.Owner], out Vector3 position, out Quaternion rotation, out Vector3 _, out Vector3 _))
				return;
			Quaternion mjToHz = owner.GetAxisHzToMj().Conjugate;
			Vector3 worldPosition = mjToHz * position + owner.GetSceneTranslation();
			Quaternion worldRotation = (mjToHz * rotation).Normalized();
			foreach ((SimRobot robot, uint body) in simObject.Bodies)
			{
				if (robot == simObject.Owner)
					continue;
				Quaternion hzToMj = robot.Mujoco.GetAxisHzToMj();
				robot.Mujoco.WeldBody(body, hzToMj * (worldPosition - robot.Mujoco.GetSceneTranslation()), (hzToMj * worldRotation).Normalized(), true);
			}
		}

		// Taking an object from the robot gripping it would drop it. Otherwise it's the nearest hand that's working, or null to leave it be.
		private SimRobot? WantedOwner(SimObject simObject)
		{
			Vector3 position = simObject.Entity.Transform.WorldTranslation;
			SimRobot? owner = simObject.Owner;
			if (owner is not null && PadForce(owner, LeftPad) > 0.0f && PadForce(owner, RightPad) > 0.0f && (HandPosition(owner) - position).Length() < GripReach)
				return owner;

			SimRobot? nearest = null;
			float nearestDistance = float.MaxValue;
			foreach (SimRobot robot in simObject.Bodies.Keys)
			{
				if (!m_Director!.IsControlled(robot.Entity))
					continue;
				float distance = (HandPosition(robot) - position).Length() - (robot == owner ? KeepOwnerBias : 0.0f);
				if (distance < nearestDistance)
				{
					nearest = robot;
					nearestDistance = distance;
				}
			}
			return nearest;
		}

		private static Vector3 HandPosition(SimRobot robot)
		{
			robot.Mujoco.GetSitePoseMj(robot.GraspSite, out Vector3 positionMj, out Quaternion _, out Vector3 _);
			return robot.Mujoco.GetAxisHzToMj().Conjugate * positionMj + robot.Mujoco.GetSceneTranslation();
		}

		// A waypoint the arm can't reach is the usual reason a move looks wrong, so it's worth saying
		private void WarnOnMiss()
		{
			foreach (SimRobot robot in m_Robots)
			{
				LD_PlateWaypoint? waypoint = m_Director!.ControlledBy(robot.Entity);
				if (waypoint != robot.LastWaypoint && robot.LastWaypoint is not null)
				{
					float miss = (HandPosition(robot) - robot.LastTarget).Length();
					if (miss > MissWarning)
						Log.Warn($"[LD_PlateSimBridge] {robot.LastWaypoint.Tag}: {robot.Entity.Tag} ended {miss * 1000.0f:0} mm from its target. It may be out of reach at this rotation, or the move too fast.");
				}
				robot.LastWaypoint = waypoint;
				robot.LastTarget = m_Director.CommandedPosition(robot.Entity);
			}
		}

		private void UpdateMotionLog(float deltaTime)
		{
			m_LogTime += deltaTime;
			m_LogWait -= deltaTime;
			if (m_LogWait > 0.0f)
				return;
			m_LogWait += LogInterval;
			for (int i = 0; i < m_Robots.Count; i++)
			{
				SimRobot robot = m_Robots[i];
				LD_PlateWaypoint? waypoint = m_Director!.ControlledBy(robot.Entity);
				if (waypoint is null && m_Director.ActiveRobot is not null)
				{
					robot.Mujoco.GetSitePoseMj(robot.GraspSite, out Vector3 _, out Quaternion site, out Vector3 _);
					Vector3 fingers = (robot.Mujoco.GetAxisHzToMj().Conjugate * site) * new Vector3(1.0f, 0.0f, 0.0f);
					Vector3 at = HandPosition(robot);
					string wrist = string.Join(" ", Array.ConvertAll(robot.JointAddresses, a => robot.Mujoco.GetJointPosition(a).ToString("0.00")));
					Log.Info($"[PlateMotion] {m_LogTime:0.00} resting{i} at {at.X:0.000} {at.Y:0.000} {at.Z:0.000} fingers {fingers.X:0.000} {fingers.Y:0.000} {fingers.Z:0.000} joints {wrist}");
					continue;
				}
				Vector3 target = m_Director.CommandedPosition(robot.Entity);
				Vector3 hand = HandPosition(robot);
				string joints = string.Join(" ", Array.ConvertAll(robot.JointAddresses, a => robot.Mujoco.GetJointPosition(a).ToString("0.000")));
				string tag = waypoint?.Tag ?? $"startup{i}";
				Log.Info($"[PlateMotion] {m_LogTime:0.00} {tag} ik target {target.X:0.000} {target.Y:0.000} {target.Z:0.000} hand {hand.X:0.000} {hand.Y:0.000} {hand.Z:0.000} joints {joints} grip {m_Director.GripperOpening(robot.Entity):0.00} fingers {FingerGap(robot) * 1000.0f:0.0}mm force L {PadForce(robot, LeftPad):0.0} R {PadForce(robot, RightPad):0.0} pads {PadPosition(robot, LeftPad)} | {PadPosition(robot, RightPad)}");
			}
			foreach (SimObject simObject in m_Objects)
			{
				if (simObject.Owner is null)
					continue;
				Vector3 entity = simObject.Entity.Transform.WorldTranslation;
				string copies = string.Join(" ", m_Robots.ConvertAll(r => simObject.Bodies.TryGetValue(r, out uint body) && r.Mujoco.GetBodyPoseMj(body, out Vector3 p, out Quaternion _, out Vector3 _, out Vector3 _)
					? $"{(r.Mujoco.GetAxisHzToMj().Conjugate * p + r.Mujoco.GetSceneTranslation()).Y:0.000}{(r == simObject.Owner ? "*" : "")}"
					: "-"));
				string up = "";
				if (simObject.Owner.Mujoco.GetBodyPoseMj(simObject.Bodies[simObject.Owner], out Vector3 _, out Quaternion rotation, out Vector3 _, out Vector3 _))
				{
					Vector3 normal = (simObject.Owner.Mujoco.GetAxisHzToMj().Conjugate * rotation) * new Vector3(0.0f, 0.0f, 1.0f);
					up = $" normal {normal.X:0.000} {normal.Y:0.000} {normal.Z:0.000}";
				}
				Log.Info($"[PlateMotion] {m_LogTime:0.00} object {simObject.Entity.Tag} entity {entity.X:0.000} {entity.Y:0.000} {entity.Z:0.000} copy heights {copies}{up}");
			}
		}

		// How far apart the finger pads are, from the finger joint's distance to fully closed on both sides
		private static float FingerGap(SimRobot robot)
		{
			float position = robot.Mujoco.GetJointPosition(robot.Mujoco.GetJointQposAdr(robot.Mujoco.GetJointID("joint7")));
			return 2.0f * MathF.Abs(position - (float)robot.GripperClosed);
		}

		private static float PadForce(SimRobot robot, string geom) => robot.Mujoco.GetGeomContactForce(robot.Mujoco.GetGeomID(geom));

		private static string PadPosition(SimRobot robot, string geom)
		{
			if (!robot.Mujoco.GetGeomPoseMj(robot.Mujoco.GetGeomID(geom), out Vector3 positionMj, out Quaternion _))
				return "-";
			Vector3 position = robot.Mujoco.GetAxisHzToMj().Conjugate * positionMj + robot.Mujoco.GetSceneTranslation();
			return $"{position.X:0.000} {position.Y:0.000} {position.Z:0.000}";
		}
	}
}
