using System;
using System.Reflection;

// The 2026.3 release names an entity's ID `ulong Entity.ID` and looks entities up with Scene.FindEntityByID(ulong).
// The local feature/plugin-sdk-pipers build renamed them to `EntityHandle Entity.Handle` and
// Scene.FindEntityByHandle(EntityHandle). Scripts go through these helpers so the project compiles on both.
namespace Hazel
{
	public static class EntityIdCompat
	{
		private static readonly FieldInfo s_IdField =
			typeof(Entity).GetField("ID", BindingFlags.Public | BindingFlags.Instance)
			?? typeof(Entity).GetField("Handle", BindingFlags.Public | BindingFlags.Instance)
			?? throw new MissingFieldException("Hazel.Entity has neither ID nor Handle.");

		private static readonly MethodInfo s_Find = FindLookup();

		private static MethodInfo FindLookup()
		{
			foreach (string name in new[] { "FindEntityByID", "FindEntityByHandle" })
			{
				MethodInfo? method = typeof(Scene).GetMethod(name, BindingFlags.Public | BindingFlags.Static);
				if (method is not null && method.GetParameters().Length == 1)
					return method;
			}
			throw new MissingMethodException("Hazel.Scene has neither FindEntityByID nor FindEntityByHandle.");
		}

		/// <summary>The entity's scene ID, the same number the scene file stores.</summary>
		public static ulong Id(this Entity entity) => Convert.ToUInt64(s_IdField.GetValue(entity));

		/// <summary>The entity with this ID, or null if there's none in the scene.</summary>
		public static Entity? Find(ulong id)
		{
			Type parameter = s_Find.GetParameters()[0].ParameterType;
			object argument = parameter.IsEnum ? Enum.ToObject(parameter, id) : id;
			return s_Find.Invoke(null, new[] { argument }) as Entity;
		}
	}
}
