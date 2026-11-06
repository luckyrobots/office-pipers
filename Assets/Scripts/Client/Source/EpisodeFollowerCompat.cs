using System;

// The WoodFiller scripts use Hazel.Data.EpisodeFollowers, which only exists in Lucky Robots' private
// development engine build. Engine builds without it can't compile this project at all, so this
// stands in for it with "no physical follower is registered", which is what those builds can offer.
//
// Delete this file on an engine that has the real API: a project type would hide the engine's, and
// episodes would no longer wait for physical followers. The check below reports that case.
namespace Hazel.Data
{
	public enum EpisodeFollowerStatus { NotRequired, Stopped }

	public static class EpisodeFollowers
	{
		static EpisodeFollowers()
		{
			if (Type.GetType("Hazel.Data.EpisodeFollowers, Hazel-ScriptCore") is not null)
				Log.Error("[EpisodeFollowerCompat] This engine has the real EpisodeFollowers API. Delete Assets/Scripts/Client/Source/EpisodeFollowerCompat.cs: while it exists, episodes don't wait for physical followers.");
		}

		public static EpisodeFollowerStatus Status(ulong worldEntityId) => EpisodeFollowerStatus.NotRequired;
		public static bool CanAdvance(ulong worldEntityId) => true;
		public static bool SetPhase(ulong worldEntityId, string episodeId, string phase) => true;
		public static bool Begin(ulong worldEntityId, string episodeId, uint ordinal) => true;
		public static void Finish(ulong worldEntityId, string episodeId, bool success, string artifact = "") { }
	}
}
