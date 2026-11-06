using System;
using System.Diagnostics;
using System.IO;
using System.Net.Sockets;
using Hazel;

namespace CanTake
{
	/// <summary>
	/// Keeps CurvePath meshes following their waypoints in Edit mode, where scripts don't run, by
	/// way of LuckyLive.exe: a small app that talks to the editor's MCP server and runs the
	/// luckylive/curve_path.py binding it finds in this package. This class only checks for it and
	/// starts it on Play; LuckyLive's window shows the rest. The editor must be started with --mcp.
	/// </summary>
	public static class CurvePathEditorSync
	{
		// LuckyLive's control API, which it holds for as long as it runs
		public const int ControlPort = 19400;

		// Shared with LuckyLive: each run writes where it is to launch.txt, so it can be started from here
		private static string Home => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "LuckyLive");
		private static string LaunchPath => Path.Combine(Home, "launch.txt");

		private static DateTime s_LastStart = DateTime.MinValue;

		/// <summary>True when something is listening on the port; doesn't wait on the editor.</summary>
		private static bool PortOpen(int port)
		{
			try
			{
				using var tcp = new TcpClient();
				return tcp.ConnectAsync("127.0.0.1", port).Wait(1000) && tcp.Connected;
			}
			catch (Exception)
			{
				return false;
			}
		}

		public static bool IsRunning() => PortOpen(ControlPort);

		/// <summary>
		/// Starts LuckyLive, minimized, unless it's already running; warns if it isn't installed. Several
		/// CurvePaths call this together on Play, so only the first call in a few seconds does anything.
		/// Started from here, LuckyLive exits a minute after the last editor closes.
		/// </summary>
		public static void EnsureRunning()
		{
			if (DateTime.Now - s_LastStart < TimeSpan.FromSeconds(10))
				return;
			s_LastStart = DateTime.Now;

			if (IsRunning())
				return;

			// LuckyLive records its location in launch.txt each time it runs, so a missing file means it has never run here
			string[] launch = File.Exists(LaunchPath) ? File.ReadAllLines(LaunchPath) : Array.Empty<string>();
			if (launch.Length == 0 || string.IsNullOrWhiteSpace(launch[0]))
			{
				Log.Warn("[CurvePath] LuckyLive isn't installed (it has never run on this machine), so curves won't follow their waypoints in Edit mode. Run LuckyLive.exe once; after that, Play starts it when needed.");
				return;
			}
			if (!File.Exists(launch[0]))
			{
				Log.Warn($"[CurvePath] LuckyLive was moved or uninstalled: {launch[0]} no longer exists, so curves won't follow their waypoints in Edit mode. Run LuckyLive.exe from its new location once to fix this.");
				return;
			}
			try
			{
				string args = (launch.Length > 1 ? launch[1] + " " : "") + "--minimized --exit-when-idle 60";
				Process.Start(new ProcessStartInfo(launch[0], args) { UseShellExecute = false, WorkingDirectory = Home });
				Log.Info("[CurvePath] Started LuckyLive (in the tray). Curves now follow their waypoints in Edit mode.");
			}
			catch (Exception e)
			{
				Log.Warn($"[CurvePath] Couldn't start LuckyLive ({launch[0]}): {e.GetBaseException().Message}");
			}
		}
	}
}
