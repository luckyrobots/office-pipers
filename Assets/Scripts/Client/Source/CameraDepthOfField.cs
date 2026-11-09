using System;
using Hazel;

namespace OfficePipers
{
	/// <summary>
	/// Depth of field while playing, focused on Focus Target: each frame the focus distance is set to the
	/// target's depth in front of this camera, so it stays sharp as either moves. Without a target it holds
	/// Focus Distance. The renderer's own depth-of-field settings are put back when you stop.
	/// This is the viewport's post blur. The path tracer instead blurs through the camera's physical lens
	/// (CameraComponent Aperture F-Stop and Focus Distance), which scripts can't set, so set those to match.
	/// </summary>
	public class CameraDepthOfField : Entity
	{
		[DisplayName("Depth Of Field")]
		[Tooltip("Blur what is nearer or further than the focus while playing.")]
		public bool Enabled = true;

		[DisplayName("Focus Target")]
		[Tooltip("Kept in focus while playing. Leave empty to use Focus Distance instead.")]
		public Entity? FocusTarget;

		[DisplayName("Focus Distance")]
		[Units("m")]
		[Slider(0.1f, 10.0f)]
		[Tooltip("Distance in front of the camera that is sharp, used when there's no Focus Target.")]
		public float FocusDistance = 1.9f;

		[DisplayName("Blur Strength")]
		[Slider(0.0f, 3.0f)]
		[Tooltip("How quickly things blur away from the focus. 0.5 softens the wall and the nearest arms a little; 1.5 is strong.")]
		public float BlurStrength = 0.5f;

		[DisplayName("Path Traced")]
		[Tooltip("Turn on when the viewport is path traced. The path tracer blurs through this camera's own lens (its Aperture F-Stop and Focus Distance), so this blur is switched off rather than blurring twice. Recordings never get this blur: a path-traced recording uses the lens, a rasterized one is sharp.")]
		public bool PathTraced = false;

		private bool m_WasEnabled;
		private float m_WasFocus;
		private float m_WasBlur;

		protected override void OnCreate()
		{
			m_WasEnabled = SceneRenderer.DepthOfField.IsEnabled;
			m_WasFocus = SceneRenderer.DepthOfField.FocusDistance;
			m_WasBlur = SceneRenderer.DepthOfField.BlurSize;
			Apply();
		}

		protected override void OnUpdate(float deltaTime) => Apply();

		protected override void OnDestroy()
		{
			SceneRenderer.DepthOfField.IsEnabled = m_WasEnabled;
			SceneRenderer.DepthOfField.FocusDistance = m_WasFocus;
			SceneRenderer.DepthOfField.BlurSize = m_WasBlur;
		}

		private void Apply()
		{
			SceneRenderer.DepthOfField.IsEnabled = Enabled && !PathTraced;
			if (!Enabled || PathTraced)
				return;
			SceneRenderer.DepthOfField.FocusDistance = MathF.Max(TargetDepth() ?? FocusDistance, 0.05f);
			SceneRenderer.DepthOfField.BlurSize = BlurStrength;
		}

		// The blur works on depth along the view, so measure along the camera's forward axis
		private float? TargetDepth()
		{
			if (FocusTarget is null || !FocusTarget)
				return null;
			Vector3 offset = FocusTarget.Transform.WorldTranslation - Transform.WorldTranslation;
			float depth = MathF.Abs(Vector3.Dot(offset, Transform.WorldTransform.Forward));
			return depth > 0.05f ? depth : offset.Length();
		}
	}
}
