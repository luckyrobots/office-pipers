using System;
using Hazel;

namespace OfficePipers
{
	/// <summary>
	/// Turntable for the demo camera: put it on Orbit_Root, with the camera as a child of Orbit_Root and
	/// Orbit_Root's origin at the table. It swings Orbit_Root about the vertical axis back and forth by
	/// Sweep Angle, easing to a stop at each end, so the child camera slowly arcs around the table without
	/// ever going round to the wall side. The swing is centred on whatever yaw Orbit_Root has in the scene.
	/// </summary>
	public class OrbitRootTurntable : Entity
	{
		[DisplayName("Sweep Angle")]
		[Units("deg")]
		[Slider(10.0f, 170.0f)]
		[Tooltip("Total arc swept end to end, centred on Orbit_Root's starting rotation. Keep it small enough that the camera never passes behind the wall.")]
		public float SweepAngle = 100.0f;

		[DisplayName("Seconds Per Sweep")]
		[Units("s")]
		[Slider(5.0f, 120.0f)]
		[Tooltip("Time to travel from one end of the arc to the other. Larger is slower.")]
		public float SecondsPerSweep = 40.0f;

		private float m_Time;
		private Quaternion m_StartRotation;

		protected override void OnCreate()
		{
			m_StartRotation = Transform.WorldRotationQuat;
		}

		protected override void OnUpdate(float deltaTime)
		{
			m_Time += deltaTime;
			float cycle = MathF.Max(SecondsPerSweep, 0.1f);
			float phase = (m_Time / cycle) % 2.0f;
			float t = phase < 1.0f ? phase : 2.0f - phase;
			float eased = t * t * (3.0f - 2.0f * t);

			float half = SweepAngle * 0.5f * (MathF.PI / 180.0f);
			float yaw = -half + 2.0f * half * eased;

			Transform.WorldRotationQuat = new Quaternion(new Vector3(0.0f, yaw, 0.0f)) * m_StartRotation;
		}
	}
}
