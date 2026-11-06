using System;
using Hazel;

namespace CanTake
{
	/// <summary>
	/// Drives the CurvePath procedural graph on this entity from its child entities: each child
	/// is a waypoint arrow, in name order, and the graph sweeps a thin tube along a Bezier curve
	/// through the arrow tips, leaving each one in the direction it points. Move or rotate a
	/// child to reshape the path; duplicate one to add a point (up to 8).
	///
	/// In a RobotDirector's Task List (the LD_Path prefab), the Path Task settings say how the
	/// director runs it: it waits Wait Start, eases the hand onto the start of the curve over Approach
	/// Time, follows the curve once to the end at Speed (with CurveFollower's motion), then waits Wait
	/// End. Grabber Action works as on LD_Waypoint, on its own timer from the start of the task.
	/// CurveFollower ignores the Path Task settings and uses its own.
	/// </summary>
	public class CurvePath : Entity
	{
		private const int MaxWaypoints = 8; // CurveFromAnchors takes 2-8 anchors
		private static readonly Vector3 ArrowAxis = Vector3.Up; // SM_Cone's tip points along local +Y

		[ToggleButton("Follow Arrows", "Auto Tangents")]
		[Tooltip("Pass through each waypoint in the direction its arrow points. Auto Tangents ignores rotation and smooths between neighbours.")]
		public bool FollowArrows = false;

		[Slider(0.0f, 3.0f)]
		[Tooltip("Handle length at each waypoint, as a multiple of a third of the neighbouring span. Higher holds each direction longer.")]
		public float Tension = 0.888f;

		[ToggleButton("Closed Loop", "Open")]
		[Tooltip("Join the last waypoint back to the first.")]
		public bool Closed = false;

		[Units("m")]
		[Slider(0.0005f, 0.05f)] // the graph's radius range
		[Tooltip("Radius of the tube drawn along the curve.")]
		public float Radius = 0.002f;

		[Tooltip("Show the curve and its waypoints while playing. Off hides both on Play; can be switched while playing. A RobotDirector's Task Visibility can override this for its curve tasks.")]
		public bool VisibleDuringSim = true;

		// The procedural mesh can't be hidden from script, so a hidden curve is drawn this far below the scene
		private static readonly Vector3 HiddenOffset = new Vector3(0.0f, -10000.0f, 0.0f);

		[Group("LuckyLive")]
		[ToggleButton("Auto-Start LuckyLive", "Manual")]
		[Tooltip("On Play, start LuckyLive.exe (in the tray) if it isn't already running, so the curve follows its waypoints in Edit mode afterwards. Logs a warning if LuckyLive isn't installed.")]
		public bool AutoStartEditSync = true; // name kept so saved scenes keep their setting

		[Group("LuckyLive")]
		[ReadOnly]
		[Tooltip("Written by LuckyLive, which keeps the curve following its waypoints in Edit mode. If it says Running but the curve doesn't follow, LuckyLive has stopped or been disconnected from this editor: check its window.")]
		public string SyncStatus = "Not connected";

		// --- How a RobotDirector runs this curve as a task (CurveFollower has its own settings) ---

		[Group("Path Task")]
		[Dropdown("Left Arm", "Right Arm")]
		[Tooltip("Which hand follows the curve. The other keeps its last target. Ignored on a single-arm robot (Panda).")]
		public int Arm = 0;

		[Group("Path Task")]
		[Units("m/s")]
		[Slider(0.01f, 2.0f)]
		[Tooltip("Speed of the hand along the curve. The arm may lag behind this at high speed.")]
		public float Speed = 2f;

		[Group("Path Task")]
		[ToggleButton("Reverse", "Forward")]
		[Tooltip("Reverse runs the curve from its last waypoint to its first.")]
		public bool Reverse = false;

		[Group("Path Task")]
		[Units("s")]
		[Slider(0.2f, 5.0f)]
		[Tooltip("Time to ease the hand from where the previous task left it onto the start of the curve.")]
		public float ApproachTime = 0.331f;

		[Group("Path Task")]
		[ToggleButton("Solve Orientation", "Position Only")]
		[Tooltip("Also control the hand's orientation along the curve, from the waypoints or a fixed Hand Orientation.")]
		public bool SolveOrientation = true;

		[Group("Path Task")]
		[EditCondition("SolveOrientation")]
		[Dropdown("Fixed", "From Waypoints")]
		[Tooltip("From Waypoints turns the hand with the curve's waypoints, blending between them: at each one the fingers point the way its arrow points (+Y) and the jaws open along its X, as with an LD_Waypoint. Fixed holds Hand Orientation.")]
		public int OrientationSource = CurveMotion.FromWaypoints;

		[Group("Path Task")]
		[Units("deg")]
		[EditCondition("SolveOrientation")]
		[Tooltip("Fixed: the hand's orientation in world XYZ degrees. From Waypoints: an extra rotation in each waypoint's own frame, on top of its arrow.")]
		public Vector3 HandOrientation = Vector3.Zero;

		[Group("Path Task")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait before this task starts.")]
		public float WaitStart = 0.0f;

		[Group("Path Task")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait at the end of the curve before the next task starts.")]
		public float WaitEnd = 0.23f;

		[Group("Path Task")]
		[DisplayName("Grabber Action")]
		[Dropdown("None", "Open", "Close", "Set")]
		[Tooltip("Opens or closes the gripper of this task's arm (Set: to Grabber Value) while the hand follows the curve: it starts Grabber Delay seconds after the task starts (the approach onto the curve) and takes Grabber Time. It runs on its own timer and never holds up the tasks.")]
		public int GrabberAction = LuckyExporter.LD_Waypoint.GrabNone;

		[Group("Path Task")]
		[DisplayName("Grabber Value")]
		[Slider(0.0f, 1.0f)]
		[Tooltip("For Set: how far open the gripper goes, from 0 (closed) to 1 (fully open), on any robot's gripper range.")]
		public float GrabberValue = 0.5f;

		[Group("Path Task")]
		[DisplayName("Grabber Delay")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Seconds after the task starts (after Wait Start, as the hand starts its Approach Time onto the curve) that the gripper starts opening or closing. 0 starts with the approach; Approach Time starts it as the hand sets off along the curve.")]
		public float GrabberDelay = 0.0f;

		[Group("Path Task")]
		[DisplayName("Grabber Time")]
		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("How long the gripper takes to open or close; 0 snaps.")]
		public float GrabberTime = 1.0f;

		/// <summary>Copies the Path Task settings into a curve motion.</summary>
		public void ApplyTask(CurveMotion motion)
		{
			motion.Arm = Arm;
			motion.Speed = Speed;
			motion.Reverse = Reverse;
			motion.ApproachTime = ApproachTime;
			motion.SolveOrientation = SolveOrientation;
			motion.OrientationSource = OrientationSource;
			motion.HandOrientation = HandOrientation;
		}

		private Vector3[] m_Last = Array.Empty<Vector3>();
		private Vector3[] m_LastAxis = Array.Empty<Vector3>();
		private bool m_LastFollow;
		private float m_LastTension;
		private bool m_LastClosed;
		private float m_LastRadius;
		private bool m_WarnedTooMany;
		private bool m_LastVisible;

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
			ApplyWaypointVisibility();
			UpdateCurve();
			// Scripts only run in Play, so this is where LuckyLive can be started for Edit mode
			if (AutoStartEditSync)
				CurvePathEditorSync.EnsureRunning();
		}

		protected override void OnUpdate(float deltaTime)
		{
			if (!StartOnce())
				return;
			// LuckyLive only runs in Edit mode, so in Play this script rebuilds the curve itself
			bool visibilityChanged = VisibleDuringSim != m_LastVisible;
			if (visibilityChanged || Changed())
			{
				ApplyWaypointVisibility(); // also catches waypoints added while playing
				UpdateCurve();
			}
		}

		private void ApplyWaypointVisibility()
		{
			m_LastVisible = VisibleDuringSim;
			// The waypoints always show or hide with the curve
			foreach (Entity child in Children)
				SetMeshesVisible(child, VisibleDuringSim);
		}

		// A waypoint can be a prefab instance (LD_Target) with its mesh on a child, so go all the way down
		private static void SetMeshesVisible(Entity entity, bool visible)
		{
			StaticMeshComponent? mesh = entity.GetComponent<StaticMeshComponent>();
			if (mesh is not null)
				mesh.Visible = visible;
			foreach (Entity child in entity.Children)
				SetMeshesVisible(child, visible);
		}

		// Push the waypoints' current positions and directions into the curve graph
		private void UpdateCurve()
		{
			PodComponent? pod = GetComponent<PodComponent>();
			if (pod is null)
			{
				Log.Warn("[CurvePath] This entity needs a Procedural Mesh component using CurvePath.hpodgraph.");
				return;
			}

			Entity[] children = Waypoints();
			int count = Math.Min(children.Length, MaxWaypoints);
			// Warn once per Play
			if (children.Length > MaxWaypoints && !m_WarnedTooMany)
			{
				Log.Warn($"[CurvePath] {children.Length} waypoints; the curve graph takes at most {MaxWaypoints}, so only the first {MaxWaypoints} (by name) are used.");
				m_WarnedTooMany = true;
			}
			if (count < 2)
			{
				Log.Warn("[CurvePath] Add at least two child entities as waypoints.");
				return;
			}

			// Children's translations are already local to this entity, which is the graph's space
			m_Last = new Vector3[count];
			m_LastAxis = new Vector3[count];
			m_LastFollow = FollowArrows;
			m_LastTension = Tension;
			m_LastClosed = Closed;
			m_LastRadius = Radius;
			Vector3 offset = VisibleDuringSim ? Vector3.Zero : HiddenOffset;
			for (int i = 0; i < count; i++)
			{
				m_Last[i] = children[i].Translation;
				m_LastAxis[i] = Axis(children[i]);
				pod.SetParameter($"anchor_{i}", m_Last[i] + offset);
				// A zero axis tells the graph to use its automatic tangent
				pod.SetParameter($"axis_{i}", FollowArrows ? m_LastAxis[i] : Vector3.Zero);
			}
			pod.SetParameter("anchor_count", count);
			pod.SetParameter("tension", Tension);
			pod.SetParameter("closed", Closed);
			pod.SetParameter("radius", Radius);
		}

		private static Vector3 Axis(Entity waypoint) => waypoint.RotationQuat * ArrowAxis;

		/// <summary>
		/// Samples the curve the graph draws (CurveBezierAligned with this script's settings), in this
		/// entity's local space, with samplesPerSegment points per span. Empty with fewer than two waypoints.
		/// </summary>
		public Vector3[] SampleLocal(int samplesPerSegment = 32)
		{
			Entity[] children = Waypoints();
			int n = Math.Min(children.Length, MaxWaypoints);
			if (n < 2)
				return Array.Empty<Vector3>();

			var p = new Vector3[n];
			var outHandle = new Vector3[n];
			var inHandle = new Vector3[n];
			for (int i = 0; i < n; i++)
				p[i] = children[i].Translation;
			for (int i = 0; i < n; i++)
			{
				Vector3 prev = Closed ? p[(i + n - 1) % n] : p[Math.Max(i - 1, 0)];
				Vector3 next = Closed ? p[(i + 1) % n] : p[Math.Min(i + 1, n - 1)];
				Vector3 axis = FollowArrows ? Axis(children[i]) : Vector3.Zero;
				// Matches the graph: handle direction from the arrow, else from the neighbours; length a third of each span
				Vector3 dir = axis.Length() > 1e-6f ? axis.Normalized() : SafeNormalize(next - prev);
				inHandle[i] = p[i] - dir * ((p[i] - prev).Length() / 3.0f * Tension);
				outHandle[i] = p[i] + dir * ((next - p[i]).Length() / 3.0f * Tension);
			}

			int segments = Closed ? n : n - 1;
			var samples = new Vector3[segments * samplesPerSegment + 1];
			int k = 0;
			for (int s = 0; s < segments; s++)
			{
				Vector3 a = p[s], c1 = outHandle[s], c2 = inHandle[(s + 1) % n], b = p[(s + 1) % n];
				int steps = s == segments - 1 ? samplesPerSegment + 1 : samplesPerSegment;
				for (int j = 0; j < steps; j++)
				{
					float t = (float)j / samplesPerSegment, u = 1.0f - t;
					samples[k++] = a * (u * u * u) + c1 * (3.0f * u * u * t) + c2 * (3.0f * u * t * t) + b * (t * t * t);
				}
			}
			return samples;
		}

		/// <summary>
		/// Rotations matching SampleLocal point for point, in this entity's local space: each waypoint's
		/// rotation, blended (slerp) across each span to the next. Empty with fewer than two waypoints.
		/// </summary>
		public Quaternion[] SampleRotationsLocal(int samplesPerSegment = 32)
		{
			Entity[] children = Waypoints();
			int n = Math.Min(children.Length, MaxWaypoints);
			if (n < 2)
				return Array.Empty<Quaternion>();

			// q and -q are the same rotation; keep neighbours in one hemisphere so each blend takes the short way
			var q = new Quaternion[n];
			for (int i = 0; i < n; i++)
			{
				q[i] = children[i].RotationQuat.Normalized();
				if (i > 0 && Dot(q[i], q[i - 1]) < 0.0f)
					q[i] = Negate(q[i]);
			}

			int segments = Closed ? n : n - 1;
			var samples = new Quaternion[segments * samplesPerSegment + 1];
			int k = 0;
			for (int s = 0; s < segments; s++)
			{
				Quaternion a = q[s], b = q[(s + 1) % n];
				if (Dot(a, b) < 0.0f)
					b = Negate(b); // the closing span of a loop
				int steps = s == segments - 1 ? samplesPerSegment + 1 : samplesPerSegment;
				for (int j = 0; j < steps; j++)
					samples[k++] = Quaternion.Slerp(a, b, (float)j / samplesPerSegment);
			}
			return samples;
		}

		private static float Dot(Quaternion a, Quaternion b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
		private static Quaternion Negate(Quaternion q) => new Quaternion(-q.X, -q.Y, -q.Z, -q.W);

		private static Vector3 SafeNormalize(Vector3 v) => v.Length() > 1e-6f ? v.Normalized() : Vector3.Zero;

		// Hierarchy order doesn't follow the outliner, so order waypoints by name (Waypoint_2 before Waypoint_10)
		private Entity[] Waypoints()
		{
			Entity[] children = Children;
			Array.Sort(children, (a, b) => NaturalCompare(a.Tag, b.Tag));
			return children;
		}

		private static int NaturalCompare(string a, string b)
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

		private bool Changed()
		{
			Entity[] children = Waypoints();
			int count = Math.Min(children.Length, MaxWaypoints);
			if (count != m_Last.Length || FollowArrows != m_LastFollow || Tension != m_LastTension || Closed != m_LastClosed || Radius != m_LastRadius)
				return true;
			for (int i = 0; i < count; i++)
			{
				if (!Vector3.EpsilonEquals(children[i].Translation, m_Last[i])
					|| !Vector3.EpsilonEquals(Axis(children[i]), m_LastAxis[i]))
					return true;
			}
			return false;
		}
	}
}
