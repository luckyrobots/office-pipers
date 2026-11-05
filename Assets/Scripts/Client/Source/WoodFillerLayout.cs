using System;

namespace OfficePipers
{
	public readonly struct TubeDeskPoint
	{
		public readonly float Left, Back;
		public TubeDeskPoint(float left, float back) { Left = left; Back = back; }
	}
	public readonly struct TubeStart
	{
		public readonly float Left, Back, YawRadians;
		public TubeStart(float left, float back, float yawRadians) { Left = left; Back = back; YawRadians = yawRadians; }
	}
	public sealed class TubeSpawnSettings
	{
		public float Width, Depth, Radius, EdgeClearance;
		public float LeftMin, LeftMax, BackMin, BackMax;
		public float GoalLeft, GoalBack, GoalExclusionRadius;
		public float MaximumBaseDistance, BaseExclusionRadius;
		public int BaseCount;
		public readonly TubeDeskPoint[] Bases = new TubeDeskPoint[4];
	}
	public static class WoodFillerLayout
	{
		private const int MaximumAttempts = 512;
		public static bool TryFixedStart(TubeSpawnSettings s, float left, float back, float yawDegrees, out TubeStart start, out string error)
		{
			start = default; error = "";
			if (!Valid(s, requireRandomRegion: false) || !float.IsFinite(left) || !float.IsFinite(back) || !float.IsFinite(yawDegrees))
			{
				error = "Invalid fixed start: use finite dimensions, coordinates and yaw, a valid goal and an assigned active base.";
				return false;
			}
			if (!InsideWorkspace(s, s.GoalLeft, s.GoalBack) || !InsideWorkspace(s, left, back))
			{
				error = "Fixed start or goal violates the desk-edge, robot-base clearance or active-robot reach bounds. No object was moved.";
				return false;
			}
			if (!OutsideGoal(s, left, back))
			{
				error = "Fixed start is too close to the place target. Increase the start-to-goal distance; no object was moved.";
				return false;
			}
			start = new TubeStart(left, back, (yawDegrees % 360f) * (MathF.PI / 180f));
			return true;
		}
		public static bool TrySample(TubeSpawnSettings s, int seed, int ordinal, out TubeStart start, out string error)
		{
			start = default; error = "";
			if (!Valid(s, requireRandomRegion: true))
			{
				error = "Invalid spawn settings: use finite dimensions, ordered bounds, a valid goal and an assigned active base.";
				return false;
			}
			float margin = s.Radius + s.EdgeClearance;
			float leftMin = MathF.Max(s.LeftMin, margin), leftMax = MathF.Min(s.LeftMax, s.Width - margin);
			float backMin = MathF.Max(s.BackMin, margin), backMax = MathF.Min(s.BackMax, s.Depth - margin);
			if (leftMin >= leftMax || backMin >= backMax || !InsideWorkspace(s, s.GoalLeft, s.GoalBack))
			{
				error = "Goal or spawn region is outside the desk/reach bounds. Adjust settings; no object was moved.";
				return false;
			}
			uint state = unchecked((uint)seed ^ (0x9E3779B9u * ((uint)ordinal + 1u)));
			if (state == 0u)
			{
				state = 0x6D2B79F5u;
			}
			for (int attempt = 0; attempt < MaximumAttempts; attempt++)
			{
				float left = leftMin + NextUnit(ref state) * (leftMax - leftMin);
				float back = backMin + NextUnit(ref state) * (backMax - backMin);
				if (OutsideGoal(s, left, back) && InsideWorkspace(s, left, back))
				{
					start = new TubeStart(left, back, NextUnit(ref state) * 2f * MathF.PI);
					return true;
				}
			}
			error = "No valid start found in 512 attempts. Increase the region or revise clearance/reach settings.";
			return false;
		}
		public static bool InsideWorkspace(TubeSpawnSettings s, float left, float back)
		{
			float margin = s.Radius + s.EdgeClearance;
			if (!float.IsFinite(left) || !float.IsFinite(back) || left < margin || left > s.Width - margin || back < margin || back > s.Depth - margin)
			{
				return false;
			}
			for (int i = 0; i < s.BaseCount; i++)
			{
				float dx = left - s.Bases[i].Left, dz = back - s.Bases[i].Back;
				float distance = dx * dx + dz * dz, exclusion = s.BaseExclusionRadius + s.Radius;
				if (distance < exclusion * exclusion || (i == 0 && distance > s.MaximumBaseDistance * s.MaximumBaseDistance))
				{
					return false;
				}
			}
			return true;
		}
		private static bool OutsideGoal(TubeSpawnSettings s, float left, float back)
		{
			float dx = left - s.GoalLeft, dz = back - s.GoalBack;
			return dx * dx + dz * dz >= s.GoalExclusionRadius * s.GoalExclusionRadius;
		}
		private static bool Valid(TubeSpawnSettings s, bool requireRandomRegion)
		{
			if (s == null || s.BaseCount < 1 || s.BaseCount > s.Bases.Length)
			{
				return false;
			}
			float[] values = { s.Width, s.Depth, s.Radius, s.EdgeClearance, s.GoalLeft, s.GoalBack, s.GoalExclusionRadius, s.MaximumBaseDistance, s.BaseExclusionRadius };
			foreach (float value in values)
			{
				if (!float.IsFinite(value))
				{
					return false;
				}
			}
			for (int i = 0; i < s.BaseCount; i++)
			{
				if (!float.IsFinite(s.Bases[i].Left) || !float.IsFinite(s.Bases[i].Back))
				{
					return false;
				}
			}
			bool validRegion = !requireRandomRegion || (float.IsFinite(s.LeftMin) && float.IsFinite(s.LeftMax)
				&& float.IsFinite(s.BackMin) && float.IsFinite(s.BackMax) && s.LeftMin < s.LeftMax && s.BackMin < s.BackMax);
			return validRegion && s.Width > 0f && s.Depth > 0f && s.Radius > 0f && s.EdgeClearance >= 0f && s.GoalExclusionRadius > 0f
				&& s.MaximumBaseDistance > s.BaseExclusionRadius + s.Radius && s.BaseExclusionRadius >= 0f;
		}
		private static float NextUnit(ref uint state)
		{
			state ^= state << 13; state ^= state >> 17; state ^= state << 5;
			return (state >> 8) * (1f / 16777216f);
		}
	}
}
