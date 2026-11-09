using Hazel;

namespace OfficePipers
{
	/// <summary>
	/// Hides this entity and everything under it while playing: meshes are switched off, text is made
	/// fully transparent and procedural graphs get their "visible" parameter turned off. Everything shows
	/// again when you stop. Put it on a group such as Measurements.
	/// </summary>
	public class HideOnPlay : Entity
	{
		[DisplayName("Hide On Play")]
		[Tooltip("While playing, hide this entity's meshes and text and everything under it. They show again when you stop.")]
		public bool Hide = true;

		protected override void OnCreate()
		{
			if (Hide)
				HideAll(this);
		}

		private static void HideAll(Entity entity)
		{
			if (entity.GetComponent<StaticMeshComponent>() is StaticMeshComponent mesh)
				mesh.Visible = false;
			if (entity.GetComponent<TextComponent>() is TextComponent text)
			{
				Vector4 color = text.Color;
				text.Color = new Vector4(color.X, color.Y, color.Z, 0.0f);
			}
			// Procedural guides follow the convention of a Bool parameter named "visible"
			if (entity.GetComponent<PodComponent>() is PodComponent pod)
				pod.SetParameter("visible", false);
			foreach (Entity child in entity.Children)
				HideAll(child);
		}
	}
}
