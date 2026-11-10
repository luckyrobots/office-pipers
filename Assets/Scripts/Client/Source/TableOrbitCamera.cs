using System;
using Hazel;

namespace OfficePipers
{
	/// <summary>
	/// Tech-demo camera that slowly sweeps back and forth along an arc around the table, always looking at
	/// the table centre. The arc covers only the front of the table (the side facing away from the wall), so
	/// the wall is always the background. It eases to a stop at each end, then sweeps back (flip-flop).
	/// Put it on the camera entity. Set Table Centre to the middle of the table top, then adjust Radius,
	/// Height and Sweep Angle. Facing Yaw says which way the viewer sits: 0 puts the camera on the +Z side
	/// looking toward -Z (wall at the -Z end), 180 mirrors it.
	/// </summary>
	public class TableOrbitCamera : Entity
	{
		[DisplayName("Table Centre")]
		[Units("m")]
		[Tooltip("World point the camera looks at. Defaults to the procedural-table's root origin (-0.835645, -0.0184698, -0.220225) raised by 0.79 m to the table surface.")]
		public Vector3 TableCentre = new Vector3(-0.835645f, -0.0184698f + 0.79f, -0.220225f);

		[DisplayName("Radius")]
		[Units("m")]
		[Slider(0.5f, 10.0f)]
		[Tooltip("Horizontal distance from Table Centre.")]
		public float Radius = 3.0f;

		[DisplayName("Height")]
		[Units("m")]
		[Slider(0.2f, 5.0f)]
		[Tooltip("Camera height above Table Centre.")]
		public float Height = 1.2f;

		[DisplayName("Sweep Angle")]
		[Units("deg")]
		[Slider(10.0f, 170.0f)]
		[Tooltip("Total arc swept end to end, centred on straight in front of the table. Keep under about 150 so the wall stays behind the table.")]
		public float SweepAngle = 110.0f;

		[DisplayName("Seconds Per Sweep")]
		[Units("s")]
		[Slider(5.0f, 120.0f)]
		[Tooltip("Time to travel from one end of the arc to the other. Larger is slower.")]
		public float SecondsPerSweep = 40.0f;

		[DisplayName("Facing Yaw")]
		[Units("deg")]
		[Tooltip("Direction from the table to the middle of the arc, in degrees around the vertical axis. 0 = camera on the +Z side, 180 = on the -Z side.")]
		public float FacingYaw = 0.0f;

		private float m_Time;

		protected override void OnUpdate(float deltaTime)
		{
			m_Time += deltaTime;
			float cycle = MathF.Max(SecondsPerSweep, 0.1f);
			// 0..1..0 triangle wave, eased with smoothstep so it slows at each end
			float phase = (m_Time / cycle) % 2.0f;
			float t = phase < 1.0f ? phase : 2.0f - phase;
			float eased = t * t * (3.0f - 2.0f * t);

			float half = SweepAngle * 0.5f * (MathF.PI / 180.0f);
			float angle = FacingYaw * (MathF.PI / 180.0f) + (-half + 2.0f * half * eased);

			// angle 0 puts the camera on +Z; positive angle swings it toward +X
			Vector3 position = new Vector3(
				TableCentre.X + Radius * MathF.Sin(angle),
				TableCentre.Y + Height,
				TableCentre.Z + Radius * MathF.Cos(angle));

			Vector3 toTarget = TableCentre - position;
			float flat = MathF.Sqrt(toTarget.X * toTarget.X + toTarget.Z * toTarget.Z);
			float yaw = MathF.Atan2(-toTarget.X, -toTarget.Z);
			float pitch = MathF.Atan2(toTarget.Y, flat);

			Transform.WorldTranslation = position;
			Transform.WorldRotationQuat = new Quaternion(new Vector3(pitch, yaw, 0.0f));
		}
	}
}
