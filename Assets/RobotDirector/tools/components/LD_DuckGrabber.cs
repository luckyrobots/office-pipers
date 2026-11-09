using System;
using Hazel;

namespace LuckyExporter
{
	/// <summary>
	/// A waypoint that goes to a random mesh instead of to where it was placed. When its turn comes
	/// (after Wait Start) it picks one of the meshes inside Box that is easy to grab and goes to its grip
	/// point, plus Offset Position. Easy means lying on its side or upright (local +Y within Pose Tolerance
	/// of horizontal or of straight up), with nothing else in the volume within Clearance of where the
	/// fingers land, and in the arm's Max Reach. The fingers always point straight down; only the turn about the vertical follows the
	/// mesh, with the jaws closing around a duck's neck. Offset Rotation adds to that. Same Target As lets
	/// later tasks reuse a pick, so one task can hover, the next descend and another lift. Everything
	/// else (Duration, Immediate, the waits, Grabber Action) works as on LD_Waypoint, because the director
	/// runs it as one.
	///
	/// The hand re-aims at the mesh every frame until the gripper closes (Follow Target). From when the
	/// gripper starts closing until it starts opening, the director locks the mesh to the hand (Fake Hold)
	/// and stops the fingers where they touch it. See RobotDirector.TryHold.
	///
	/// Box is any entity with the default cube mesh; its transform sets the volume, which can be
	/// rotated and can be hidden. A mesh is inside when its entity's origin is, so a mesh whose origin
	/// is at its base counts by its base. The robot, the Task List, the director and Box itself are
	/// never candidates. Any other visible mesh lying on its side is, so set Tag Filter if the volume
	/// also holds something the hand shouldn't go for.
	///
	/// The Ld_DuckGrabber prefab carries this script. As with LD_Waypoint, LuckyEngine's prefab editor
	/// can't load project scripts and may crash on it, so adjust the instances in the scene.
	/// </summary>
	public class LD_DuckGrabber : LD_Waypoint
	{
		private const float BoxHalfSize = 0.5f; // the default cube mesh spans -0.5 to 0.5 on each axis
		private const float MaxPoseTolerance = 45.0f; // the on-side and upright ranges meet here
		private const float MaxHeadDownPart = -0.05f; // the lowest a mesh's local +Y may point, about 3 degrees below horizontal
		private const float FingerReach = 0.05f; // from the grip point to a fully open finger, along the jaws

		private static readonly System.Random s_Random = new();

		[Tooltip("An entity with the default cube mesh. Meshes whose origin is inside this volume when the task starts are the candidates.")]
		public Entity? Box;

		[DisplayName("Tag Filter")]
		[Tooltip("Only meshes with this text in their name are candidates, for example rubber_duck. Empty allows any mesh in the volume.")]
		public string TagFilter = "rubber_duck";

		[DisplayName("Same Target As")]
		[Tooltip("Another LD_DuckGrabber earlier in the Task List. This task uses that pick and its hand rotation instead of picking, so Box and the filters here are ignored. Use it to descend to, or lift, the mesh a hover task chose.")]
		public Entity? SameTargetAs;

		[DisplayName("Pose Tolerance")]
		[Units("deg")]
		[Slider(0.0f, 45.0f)]
		[Tooltip("How far a mesh's local +Y can be from horizontal (on its side) or straight up (upright) to count as grabbable. Meshes in any other pose, such as upside down or on their side with the +Y end tipped down, are skipped.")]
		public float PoseTolerance = 20.0f;

		[Units("m")]
		[Slider(0.0f, 0.15f)]
		[Tooltip("Skip a mesh if another mesh in the volume comes this close, seen from above, to where the fingers land. Measured from the other mesh's axis, so include its radius. 0 turns this off.")]
		public float Clearance = 0.05f;

		[DisplayName("Max Reach")]
		[Units("m")]
		[Slider(0.0f, 1.5f)]
		[Tooltip("Skip a mesh if its grip point is further than this from the robot, seen from above: with the fingers straight down, an arm can't stretch as far as it otherwise could. The default suits Panda at a table. 0 turns this off.")]
		public float MaxReach = 0.71f;

		[DisplayName("Grip Point")]
		[Tooltip("Where the fingertips go on a mesh lying on its side, in its own unscaled coordinates, so it holds for any size of mesh. The default is the rubber duck's neck, as low as the open fingers reach without touching it: lower down, its chest and back leave the jaws only millimetres to spare.")]
		public Vector3 GripPoint = new Vector3(0f, 0.145f, 0.017f);

		[DisplayName("Side Jaw Angle")]
		[Units("deg")]
		[Slider(-45.0f, 45.0f)]
		[Tooltip("On a mesh lying on its side, the line the jaws close along, measured from the mesh's +Z towards its +Y. The default fits the rubber duck's neck.")]
		public float SideJawAngle = 15.0f;

		[DisplayName("Upright Grip Point")]
		[Tooltip("Where the fingertips go on an upright mesh, in its own unscaled coordinates. The default is the rubber duck's neck, just above its body.")]
		public Vector3 UprightGripPoint = new Vector3(0.0f, 0.105f, 0.024f);

		[DisplayName("Offset Position")]
		[Units("m")]
		[Tooltip("An adjustment from the grip point in metres, in the frame after Offset Rotation: +Y is down along the fingers, so -Y hovers above, and X is the way the jaws open.")]
		public Vector3 OffsetPosition = new Vector3(0.0f, 0.0f, 0.0f);

		[DisplayName("Offset Rotation")]
		[Units("deg")]
		[Tooltip("Added to the top-down rotation, in this entity's own frame. Y turns the hand about the fingers; X and Z tilt it away from straight down.")]
		public Vector3 OffsetRotation = new Vector3(0f, 0f, 0f);

		[DisplayName("Follow Target")]
		[ToggleButton("Follow", "Fixed")]
		[Tooltip("Re-reads the mesh's pose every frame, so the hand keeps aiming at the grip point if the mesh rolls or settles, until the gripper starts closing. Fixed goes to where the mesh was when picked. Followers use the setting of the task they share a target with.")]
		public bool FollowTarget = true;

		[DisplayName("Fake Hold")]
		[ToggleButton("Hold", "Physics Only")]
		[Tooltip("Locks the mesh to the hand from when the gripper starts closing until it starts opening, so it can't slip or drop. Physics Only leaves it to the fingers' friction. Applies if the hand is near the grip point when it closes.")]
		public bool FakeHold = true;

		[DisplayName("Target Marker")]
		[ToggleButton("Show", "Hide")]
		[Tooltip("During Play, a label of the chosen mesh's name and the position and rotation the hand is sent to, including the offsets. It stays until this task picks again. A director's Task Visibility of Hide All hides it too.")]
		public bool ShowMarker = true;

		[DisplayName("Marker Lines")]
		[ToggleButton("Show", "Hide")]
		[EditCondition("ShowMarker")]
		[Tooltip("Adds to the label: axes at the point the hand is sent to, turned to its rotation, and lines from there to the chosen mesh and the label.")]
		public bool ShowMarkerLines = false;

		[DisplayName("Marker Size")]
		[Units("m")]
		[Slider(0.01f, 0.2f)]
		[EditCondition("ShowMarker")]
		[Tooltip("Sets the label's size and height: it sits four times this above the point the hand is sent to, so it stands clear of a pile. Also the axes' reach with Marker Lines.")]
		public float MarkerSize = 0.03f;

		private const float LabelScalePerSize = 1.0f; // at the default size a line is about 15 pixels tall from 1.7 m in a 900-pixel viewport
		private const float LabelHeightPerSize = 4.0f;
		private const float DegPerRad = 180.0f / MathF.PI;
		private static readonly Vector4 s_MarkerColor = new Vector4(0.1f, 0.9f, 1.0f, 1.0f); // cyan: the LD markers' orange is lost against the ducks

		internal bool MarkerHidden; // set by the director for Hide All

		private Entity? m_Target; // null until the first pick, so a prefab preview's copy never calls the engine
		private Entity? m_Label;
		private TextComponent? m_LabelText;
		private Entity? m_Camera;
		private LD_DuckGrabber? m_Source; // the task that made the pick: this one, or the one Same Target As leads to
		private bool m_FollowEnded; // the gripper has closed on this pick
		private Vector3 m_PlacedJaws; // the jaws' direction as placed in the scene, set at the first pick
		private bool m_HasPlacedJaws;
		private Vector3 m_Grip; // the last pick's grip point in the world
		private Quaternion m_Frame; // the frame Offset Position was measured in
		private Quaternion m_Hand; // the rotation given to the hand: m_Frame, turned if that's kinder to the wrist

		protected override void OnUpdate(float deltaTime)
		{
			if (m_Target is null)
				return;
			bool show = ShowMarker && !MarkerHidden && m_Target;
			if (m_LabelText is not null && !show)
				m_LabelText.Text = "";
			if (show)
				DrawMarker(m_Target);
		}

		private void DrawMarker(Entity target)
		{
			// This entity is the hand's goal: the mesh's pose, with the offsets, from when it was picked
			Vector3 goal = Transform.WorldTranslation;
			Quaternion rotation = Transform.WorldRotationQuat;
			Vector3 labelAt = goal + new Vector3(0.0f, MarkerSize * LabelHeightPerSize, 0.0f);

			if (ShowMarkerLines)
			{
				DrawAxis(goal, rotation * new Vector3(MarkerSize, 0.0f, 0.0f));
				DrawAxis(goal, rotation * new Vector3(0.0f, MarkerSize, 0.0f));
				DrawAxis(goal, rotation * new Vector3(0.0f, 0.0f, MarkerSize));
				DebugRenderer.DrawLine(goal, labelAt, s_MarkerColor);
				DebugRenderer.DrawLine(goal, target.Transform.WorldTranslation, s_MarkerColor);
			}

			if (m_Label is null)
			{
				m_Label = Scene.CreateEntity($"{Tag} Target Label");
				m_LabelText = m_Label.CreateComponent<TextComponent>();
				m_LabelText!.Color = s_MarkerColor;
				foreach (Entity entity in Scene.GetEntities())
				{
					if (entity.GetComponent<CameraComponent>() is not null)
					{
						m_Camera = entity;
						break;
					}
				}
			}

			float scale = MarkerSize * LabelScalePerSize;
			m_Label.Transform.WorldTranslation = labelAt;
			m_Label.Transform.Scale = new Vector3(scale, scale, scale);
			if (m_Camera is not null)
			{
				// Turn the text's front (+Z) to the scene's camera, keeping it upright. Scripts can't read the
				// editor's free camera, so from that view the text looks slanted.
				Vector3 toCamera = m_Camera.Transform.WorldTranslation - labelAt;
				float flat = MathF.Sqrt(toCamera.X * toCamera.X + toCamera.Z * toCamera.Z);
				m_Label.Transform.Rotation = new Vector3(-MathF.Atan2(toCamera.Y, flat), MathF.Atan2(toCamera.X, toCamera.Z), 0.0f);
			}
			Vector3 degrees = rotation.EulerAngles * DegPerRad;
			m_LabelText!.Text = $"{target.Tag}\n{goal.X:0.000}, {goal.Y:0.000}, {goal.Z:0.000}\n{degrees.X:0}, {degrees.Y:0}, {degrees.Z:0} deg";
		}

		private static void DrawAxis(Vector3 centre, Vector3 reach)
			=> DebugRenderer.DrawLine(centre - reach, centre + reach, s_MarkerColor);

		/// <summary>Moves and turns this entity to the grip point of a random easy candidate, or of Same Target As's pick, plus the offsets. False if there's no candidate, and it stays where it is.</summary>
		internal bool PickTarget(params Entity?[] excluded)
		{
			// An entity field saved unset arrives in Play as an invalid Entity rather than null, so test validity
			if (HasEntity(SameTargetAs))
				return FollowPick();

			if (!HasEntity(Box))
			{
				Log.Warn($"[LD_DuckGrabber] {Tag}: set Box to an entity with the default cube mesh. Skipping this task.");
				return false;
			}

			var inVolume = new System.Collections.Generic.List<Entity>();
			foreach (Entity entity in Scene.GetEntities())
				if (IsCandidate(entity, excluded))
					inVolume.Add(entity);

			var candidates = new System.Collections.Generic.List<Entity>();
			foreach (Entity entity in inVolume)
				if (IsEasyPose(entity) && HasRoomForFingers(entity, inVolume) && WithinReach(entity, excluded))
					candidates.Add(entity);

			if (candidates.Count == 0)
			{
				Log.Warn($"[LD_DuckGrabber] {Tag}: none of the {inVolume.Count} meshes inside {Box.Tag}{(TagFilter.Length > 0 ? $" with '{TagFilter}' in their name" : "")} is easy to grab. Skipping this task.");
				return false;
			}

			if (!m_HasPlacedJaws)
			{
				m_PlacedJaws = Transform.WorldRotationQuat * new Vector3(1.0f, 0.0f, 0.0f);
				m_HasPlacedJaws = true;
			}

			Entity target = candidates[s_Random.Next(candidates.Count)];
			Log.Info($"[LD_DuckGrabber] {Tag}: picked {target.Tag}, {(IsUpright(target) ? "upright" : "on its side")}, from {candidates.Count} easy of {inVolume.Count} in the volume.");

			m_Target = target;
			m_Source = this;
			m_FollowEnded = false;
			AimAtTarget(m_PlacedJaws);
			ApplyPick();
			return true;
		}

		/// <summary>With Same Target As, moves this entity to that task's pick ahead of this task starting. Otherwise it stays where it is, as this task hasn't picked yet.</summary>
		internal void AimAtSharedTarget()
		{
			if (HasEntity(SameTargetAs))
				FollowPick();
		}

		/// <summary>The mesh this task last went for, and whether the director should hold it when the gripper closes.</summary>
		internal Entity? Target => m_Target;
		internal bool HoldsTarget => (m_Source ?? this).FakeHold;
		internal bool Follows => (m_Source ?? this).FollowTarget;
		internal bool StillFollowing => Follows && !(m_Source ?? this).m_FollowEnded;

		/// <summary>Ends following until the next pick, as the gripper has closed on the mesh.</summary>
		internal void StopFollowing() => (m_Source ?? this).m_FollowEnded = true;

		/// <summary>The grip point, as last read from the mesh.</summary>
		internal Vector3 GripWorld => m_Grip;

		/// <summary>Re-reads the mesh's pose and moves this entity to match, for a hand following it.</summary>
		internal void FollowCurrentPose()
		{
			LD_DuckGrabber source = m_Source ?? this;
			if (source.m_Target is null || !source.m_Target)
				return;
			// Kept on the task that picked, so a later task sharing the target starts from the latest pose
			source.AimAtTarget(source.m_Hand * new Vector3(1.0f, 0.0f, 0.0f));
			m_Grip = source.m_Grip;
			m_Frame = source.m_Frame;
			m_Hand = source.m_Hand;
			ApplyPick();
		}

		private void AimAtTarget(Vector3 jawsNow)
		{
			m_Grip = GripPosition(m_Target!);
			m_Frame = GripFrame(m_Target!);
			m_Hand = m_Frame;

			// The jaws are the same either way round, so take the way that turns the wrist least
			Vector3 jaws = m_Frame * new Vector3(1.0f, 0.0f, 0.0f);
			if (jaws.X * jawsNow.X + jaws.Y * jawsNow.Y + jaws.Z * jawsNow.Z < 0.0f)
				m_Hand = (m_Frame * Quaternion.AngleAxis(180.0f, new Vector3(0.0f, 1.0f, 0.0f))).Normalized();
		}

		private static bool HasEntity(Entity? entity) => entity is not null && entity;

		private bool FollowPick()
		{
			LD_DuckGrabber? source = SameTargetAs!.As<LD_DuckGrabber>();
			if (source?.m_Target is null)
			{
				Log.Warn($"[LD_DuckGrabber] {Tag}: Same Target As needs an LD_DuckGrabber earlier in the Task List that has picked a mesh. Skipping this task.");
				return false;
			}
			m_Target = source.m_Target;
			m_Source = source.m_Source ?? source;
			m_Grip = source.m_Grip;
			m_Frame = source.m_Frame;
			m_Hand = source.m_Hand;
			ApplyPick();
			return true;
		}

		private void ApplyPick()
		{
			Transform.WorldTranslation = m_Grip + m_Frame * OffsetPosition;
			Transform.WorldRotationQuat = m_Hand;
		}

		private bool IsUpright(Entity entity) => (entity.Transform.WorldRotationQuat * new Vector3(0.0f, 1.0f, 0.0f)).Y > 0.0f
			&& !OnItsSide(entity);

		private bool IsEasyPose(Entity entity)
		{
			float tolerance = Math.Clamp(PoseTolerance, 0.0f, MaxPoseTolerance) / DegPerRad;
			float verticalPart = (entity.Transform.WorldRotationQuat * new Vector3(0.0f, 1.0f, 0.0f)).Y;
			// Head-down on its side, the neck is below the body, which stops fingers coming from above
			if (verticalPart < MaxHeadDownPart)
				return false;
			return MathF.Abs(verticalPart) <= MathF.Sin(tolerance) || verticalPart >= MathF.Cos(tolerance);
		}

		// Closer to horizontal than to straight up or down
		private bool OnItsSide(Entity entity)
			=> MathF.Abs((entity.Transform.WorldRotationQuat * new Vector3(0.0f, 1.0f, 0.0f)).Y) < MathF.Sin(MaxPoseTolerance / DegPerRad);

		private Vector3 GripPosition(Entity target)
		{
			Vector3 point = IsUpright(target) ? UprightGripPoint : GripPoint;
			Vector3 scale = target.Transform.WorldScale;
			return target.Transform.WorldTranslation
				+ target.Transform.WorldRotationQuat * new Vector3(point.X * scale.X, point.Y * scale.Y, point.Z * scale.Z);
		}

		// Fingers down (180 about X), turned about the vertical to suit the mesh's pose
		private Quaternion GripFrame(Entity target)
		{
			Quaternion rotation = target.Transform.WorldRotationQuat;
			float heading;
			if (IsUpright(target))
			{
				Vector3 side = rotation * new Vector3(1.0f, 0.0f, 0.0f);
				heading = MathF.Atan2(-side.Z, side.X);
			}
			else
			{
				float angle = SideJawAngle / DegPerRad;
				Vector3 jaws = rotation * new Vector3(0.0f, MathF.Sin(angle), MathF.Cos(angle));
				Vector3 up = rotation * new Vector3(0.0f, 1.0f, 0.0f);
				// On its back or front that line is near vertical, so close across the mesh's +Y instead
				heading = jaws.X * jaws.X + jaws.Z * jaws.Z > 0.25f ? MathF.Atan2(-jaws.Z, jaws.X) : MathF.Atan2(-up.X, -up.Z);
			}
			return (Quaternion.AngleAxis(heading * DegPerRad, new Vector3(0.0f, 1.0f, 0.0f))
				* new Quaternion(new Vector3(MathF.PI, 0.0f, 0.0f))
				* new Quaternion(OffsetRotation / DegPerRad)).Normalized();
		}

		// The director passes the robot first among the entities that aren't candidates
		private bool WithinReach(Entity target, Entity?[] excluded)
		{
			if (MaxReach <= 0.0f || excluded.Length == 0 || !HasEntity(excluded[0]))
				return true;
			Vector3 fromRobot = GripPosition(target) - excluded[0]!.Transform.WorldTranslation;
			return fromRobot.X * fromRobot.X + fromRobot.Z * fromRobot.Z <= MaxReach * MaxReach;
		}

		private bool HasRoomForFingers(Entity target, System.Collections.Generic.List<Entity> others)
		{
			if (Clearance <= 0.0f)
				return true;
			Vector3 grip = GripPosition(target);
			Vector3 jaws = GripFrame(target) * new Vector3(FingerReach, 0.0f, 0.0f);
			foreach (Entity other in others)
			{
				if (other.Id() == target.Id())
					continue;
				Vector3 from = other.Transform.WorldTranslation;
				Vector3 to = GripPosition(other);
				if (FlatDistanceToSegment(grip + jaws, from, to) < Clearance || FlatDistanceToSegment(grip - jaws, from, to) < Clearance)
					return false;
			}
			return true;
		}

		// Seen from above, how far a point is from the segment between two others
		private static float FlatDistanceToSegment(Vector3 point, Vector3 from, Vector3 to)
		{
			float dx = to.X - from.X, dz = to.Z - from.Z;
			float lengthSquared = dx * dx + dz * dz;
			float t = lengthSquared > 0.0f ? Math.Clamp(((point.X - from.X) * dx + (point.Z - from.Z) * dz) / lengthSquared, 0.0f, 1.0f) : 0.0f;
			float px = point.X - (from.X + dx * t), pz = point.Z - (from.Z + dz * t);
			return MathF.Sqrt(px * px + pz * pz);
		}

		private bool IsCandidate(Entity entity, Entity?[] excluded)
		{
			if (entity.Id() == this.Id() || IsUnder(entity, Box))
				return false;
			foreach (Entity? root in excluded)
				if (IsUnder(entity, root))
					return false;

			StaticMeshComponent? mesh = entity.GetComponent<StaticMeshComponent>();
			if (mesh is null || !mesh.Visible)
				return false;
			if (TagFilter.Length > 0 && !entity.Tag.Contains(TagFilter, StringComparison.OrdinalIgnoreCase))
				return false;
			return InsideBox(entity.Transform.WorldTranslation);
		}

		private bool InsideBox(Vector3 worldPosition)
		{
			Vector3 local = Box!.Transform.WorldRotationQuat.Conjugate * (worldPosition - Box.Transform.WorldTranslation);
			Vector3 scale = Box.Transform.WorldScale;
			return MathF.Abs(local.X) <= BoxHalfSize * MathF.Abs(scale.X)
				&& MathF.Abs(local.Y) <= BoxHalfSize * MathF.Abs(scale.Y)
				&& MathF.Abs(local.Z) <= BoxHalfSize * MathF.Abs(scale.Z);
		}

		// True if entity is root or anywhere below it
		private static bool IsUnder(Entity entity, Entity? root)
		{
			if (root is null)
				return false;
			for (Entity? e = entity; e is not null; e = e.Parent)
				if (e.Id() == root.Id())
					return true;
			return false;
		}
	}
}
