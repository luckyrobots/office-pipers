using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text.Json;
using System.Threading.Tasks;
using System.Xml;
using System.Xml.Linq;
using Hazel;

namespace OfficePipers
{
	public sealed class TubeReplaySample
	{
		public double TimeSeconds { get; init; }
		public float[] JointDegrees { get; init; } = Array.Empty<float>();
		public float GripperOpeningMm { get; init; }
		public float[] CommandedJointDegrees { get; init; } = Array.Empty<float>();
		public float CommandedGripperOpeningMm { get; init; }
		public ulong SyncIndex { get; init; }
		public string Source => "simulation-measured-qpos";
		public string CommandSource => "simulation-position-actuator-ctrl";
	}

	public sealed class TubeEpisodeExport
	{
		public string Kind { get; init; } = "lucky.piper.joint-trajectory";
		public int Version => 2;
		public string Mode => "simulation";
		public string ArtifactId { get; init; } = "";
		public string SessionId { get; init; } = "";
		public string CreatedUtc { get; init; } = "";
		public int Seed { get; init; }
		public int EpisodeOrdinal { get; init; }
		public object Robot { get; init; } = new();
		public object Desk { get; init; } = new();
		public object TubeDimensions { get; init; } = new();
		public object TubeStart { get; init; } = new();
		public object Goal { get; init; } = new();
		public object NativeRecording { get; init; } = new();
		public object InitialSimulationState { get; init; } = new();
		public string Outcome { get; set; } = "not-finished";
		public string OutcomeSource => "completion-request-and-simulated-placement-checks; native-recorder-finalization-unverified";
		public string CommandTimestampConvention => "postphysics-validation-samples-zero-order-hold-v1";
		public int NominalCaptureHz => 50;
		public TubeReplaySample[] Samples { get; set; } = Array.Empty<TubeReplaySample>();
	}

	public sealed class TubeActuatorQualification
	{
		public const string PinnedSourceSha256 = "da9c70def2b1b342b96f5455f8d384e3fe39c54450b227555c07d2795457a3cc";
		public const string LegacyQualifiedSourceSha256 = "63a5ba5b6be49eeeb6d216594714255d61a8d50b41d4ef799fb3b8f20e519163";
		public const string ValidationDescription = "verified authored position tags for pinned task asset; runtime actuator-law type not introspectable";
		public const string RuntimeChecksDescription = "exact actuator names, ordered joint transmissions, control ranges and policy ownership";
		public const string RuntimeTuningDescription = "native Play may override gains while retaining position-controller semantics";
		public bool Verified { get; private init; }
		public string SourceSha256 { get; private init; } = "";

		public static bool IsQualifiedSource(string? sha256)
		{
			return sha256 == PinnedSourceSha256 || sha256 == LegacyQualifiedSourceSha256;
		}

		public static Task<TubeActuatorQualification> VerifyAsync(string sourcePath)
		{
			return Task.Run(() => Verify(sourcePath));
		}

		private static TubeActuatorQualification Verify(string sourcePath)
		{
			if (string.IsNullOrWhiteSpace(sourcePath) || !Path.IsPathFullyQualified(sourcePath))
			{
				throw new InvalidDataException("Choose the exact pinned task model XML source before preparing an episode.");
			}
			using var stream = new FileStream(sourcePath, FileMode.Open, FileAccess.Read, FileShare.Read);
			if (stream.Length > 4 * 1024 * 1024)
			{
				throw new InvalidDataException("Task source exceeds the qualified asset size bound.");
			}
			string hash = Convert.ToHexString(SHA256.HashData(stream)).ToLowerInvariant();
			if (!IsQualifiedSource(hash))
			{
				throw new InvalidDataException("Task model source SHA-256 differs from the qualified position-actuator asset. Requalify a changed model deliberately.");
			}
			stream.Position = 0;
			using var reader = XmlReader.Create(stream, new XmlReaderSettings { DtdProcessing = DtdProcessing.Prohibit, XmlResolver = null });
			XDocument document = XDocument.Load(reader);
			XElement[] actuators = document.Root?.Element("actuator")?.Elements().ToArray() ?? Array.Empty<XElement>();
			string[] expected = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper" };
			if (actuators.Length != expected.Length)
			{
				throw new InvalidDataException("Qualified task requires exactly seven position actuators.");
			}
			for (int i = 0; i < expected.Length; i++)
			{
				string expectedJoint = i == 6 ? "joint7" : expected[i];
				if (actuators[i].Name != "position" || (string?)actuators[i].Attribute("name") != expected[i] || (string?)actuators[i].Attribute("joint") != expectedJoint)
				{
					throw new InvalidDataException("Task actuator tags/order/transmissions are not the qualified position-control recipe.");
				}
			}
			return new TubeActuatorQualification { Verified = true, SourceSha256 = hash };
		}
	}

	public sealed class WoodFillerTrajectory
	{
		private const int MaximumSamples = 18001;
		private const float SlideTolerance = 0.00005f;
		private const float EqualitySymmetryTolerance = 0.001f;
		private const float OpeningRoundingToleranceMm = 0.0001f;
		private static readonly JsonSerializerOptions JsonOptions = new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase, WriteIndented = true };
		private readonly uint[] m_ArmAddresses = new uint[6];
		private readonly uint[] m_ArmActuators = new uint[6];
		private readonly float[] m_CommandMinimum = new float[6];
		private readonly float[] m_CommandMaximum = new float[6];
		private readonly float[] m_Controls = new float[7];
		private readonly List<TubeReplaySample> m_Samples = new(MaximumSamples);
		private readonly string[] m_JointNames = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
		private uint m_FirstFinger;
		private uint m_SecondFinger;
		private uint m_GripperActuator;
		private float m_Travel;
		private float m_FirstMin, m_FirstMax, m_SecondMin, m_SecondMax;
		private double m_StartTime;
		private ulong m_LastSync;
		public float GripperReferenceGapMm => 2000f * m_Travel;
		public float MaximumOpeningMm => MathF.Min(90f, GripperReferenceGapMm + 1000f * (m_FirstMax - m_SecondMin));
		public int Count => m_Samples.Count;
		public float MaximumObservedFingerAsymmetryMm { get; private set; }

		public bool Configure(MujocoSceneComponent world, TubeActuatorQualification qualification, out string error)
		{
			error = "";
			if (!qualification.Verified || !TubeActuatorQualification.IsQualifiedSource(qualification.SourceSha256) || world.ModelNu != 7)
			{
				error = "Command capture requires the pinned seven-position-actuator task asset, qualified before preparation.";
				return false;
			}
			float[] hardwareMinimum = { -150f, 0f, -170f, -100f, -70f, -170f };
			float[] hardwareMaximum = { 150f, 180f, 0f, 100f, 70f, 170f };
			for (int i = 0; i < m_JointNames.Length; i++)
			{
				uint joint = world.GetJointID(m_JointNames[i]);
				if (joint == uint.MaxValue || world.GetJointType(joint) != 3)
				{
					error = "Trajectory export requires the six canonical Piper hinge joints joint1 through joint6 in the same world.";
					return false;
				}
				m_ArmAddresses[i] = world.GetJointQposAdr(joint);
				uint actuator = world.GetActuatorIndex(m_JointNames[i]);
				if (actuator != (uint)i || world.GetActuatorTrnJointID(actuator) != (int)joint || world.IsActuatorClaimedByPolicy(actuator)
					|| !world.GetJointRange(joint, out float jointMin, out float jointMax) || !world.GetActuatorCtrlRange(actuator, out float ctrlMin, out float ctrlMax)
					|| !float.IsFinite(jointMin) || !float.IsFinite(jointMax) || !float.IsFinite(ctrlMin) || !float.IsFinite(ctrlMax))
				{
					error = "Position actuator mapping/order, transmission, ranges or policy ownership differs from the qualified task asset.";
					return false;
				}
				m_ArmActuators[i] = actuator;
				m_CommandMinimum[i] = MathF.Max(MathF.Max(jointMin, ctrlMin), hardwareMinimum[i] * MathF.PI / 180f);
				m_CommandMaximum[i] = MathF.Min(MathF.Min(jointMax, ctrlMax), hardwareMaximum[i] * MathF.PI / 180f);
			}
			uint first = world.GetJointID("joint7"), second = world.GetJointID("joint8");
			if (first == uint.MaxValue || second == uint.MaxValue || world.GetJointType(first) != 2 || world.GetJointType(second) != 2
				|| !world.GetJointRange(first, out float firstMin, out float firstMax) || !world.GetJointRange(second, out float secondMin, out float secondMax)
				|| !float.IsFinite(firstMin) || !float.IsFinite(firstMax) || !float.IsFinite(secondMin) || !float.IsFinite(secondMax)
				|| firstMin >= firstMax || firstMax > 0.000001f || secondMin < -0.000001f || secondMin >= secondMax || firstMin < -0.050001f || secondMax > 0.050001f
				|| MathF.Abs(firstMin + secondMax) > 0.000001f || MathF.Abs(firstMax + secondMin) > 0.000001f)
			{
				error = "Trajectory export requires canonical opposite joint7/joint8 Pika slides, open at zero and with equal travel.";
				return false;
			}
			m_FirstFinger = world.GetJointQposAdr(first); m_SecondFinger = world.GetJointQposAdr(second); m_Travel = secondMax;
			m_FirstMin = firstMin; m_FirstMax = firstMax; m_SecondMin = secondMin; m_SecondMax = secondMax;
			m_GripperActuator = world.GetActuatorIndex("gripper");
			if (m_GripperActuator != 6 || world.GetActuatorTrnJointID(m_GripperActuator) != (int)first || world.IsActuatorClaimedByPolicy(m_GripperActuator)
				|| !world.GetActuatorCtrlRange(m_GripperActuator, out float gripperMin, out float gripperMax)
				|| MathF.Abs(gripperMin - firstMin) > 0.000001f || MathF.Abs(gripperMax - firstMax) > 0.000001f || MaximumOpeningMm <= 0f)
			{
				error = "The gripper position actuator does not match the qualified limited joint7 transmission.";
				return false;
			}
			return true;
		}

		public bool Begin(MujocoSceneComponent world, ulong syncIndex, out string error)
		{
			m_Samples.Clear(); m_StartTime = world.MujocoTime; m_LastSync = syncIndex;
			MaximumObservedFingerAsymmetryMm = 0f;
			return Capture(world, syncIndex, initial: true, out error);
		}

		public bool Capture(MujocoSceneComponent world, ulong syncIndex, bool initial, out string error)
		{
			error = "";
			if (m_Samples.Count >= MaximumSamples)
			{
				error = "Trajectory buffer reached 18,001 samples. Finish this episode; capture cannot silently discard samples.";
				return false;
			}
			double time = initial ? 0d : world.MujocoTime - m_StartTime;
			if (!initial && m_Samples.Count > 0 && time == m_Samples[^1].TimeSeconds && syncIndex >= m_LastSync)
			{
				return true;
			}
			if (!double.IsFinite(time) || (m_Samples.Count > 0 && (time <= m_Samples[^1].TimeSeconds || syncIndex < m_LastSync)))
			{
				error = "Simulation time did not advance monotonically. This trajectory cannot be exported as valid replay data.";
				return false;
			}
			float[] degrees = new float[6];
			float[] commandedDegrees = new float[6];
			if (world.ModelNu != m_Controls.Length || world.GetFullCtrl(m_Controls) != m_Controls.Length)
			{
				error = "Could not capture the complete native position-control vector.";
				return false;
			}
			for (int i = 0; i < degrees.Length; i++)
			{
				degrees[i] = world.GetJointPosition(m_ArmAddresses[i]) * 180f / MathF.PI;
				float command = m_Controls[m_ArmActuators[i]];
				if (!float.IsFinite(degrees[i]) || !float.IsFinite(command) || command < m_CommandMinimum[i] || command > m_CommandMaximum[i]
					|| world.IsActuatorClaimedByPolicy(m_ArmActuators[i]))
				{
					error = "A simulated joint observation/position command is invalid, outside the task/hardware limits, or owned by a policy.";
					return false;
				}
				commandedDegrees[i] = command * 180f / MathF.PI;
			}
			float q7 = world.GetJointPosition(m_FirstFinger), q8 = world.GetJointPosition(m_SecondFinger);
			if (!float.IsFinite(q7) || !float.IsFinite(q8) || q7 < m_FirstMin - SlideTolerance || q7 > m_FirstMax + SlideTolerance
				|| q8 < m_SecondMin - SlideTolerance || q8 > m_SecondMax + SlideTolerance || MathF.Abs(q7 + q8) > EqualitySymmetryTolerance)
			{
				error = "The simulated Pika slides are asymmetric or outside their verified limits.";
				return false;
			}
			float opening = GripperReferenceGapMm + 1000f * (q7 - q8);
			float gripperCommand = m_Controls[m_GripperActuator];
			float commandedOpening = GripperReferenceGapMm + 2000f * gripperCommand;
			MaximumObservedFingerAsymmetryMm = MathF.Max(MaximumObservedFingerAsymmetryMm, MathF.Abs(q7 + q8) * 1000f);
			if (opening < -OpeningRoundingToleranceMm || opening > MaximumOpeningMm + OpeningRoundingToleranceMm
				|| !float.IsFinite(gripperCommand) || gripperCommand < m_FirstMin || gripperCommand > m_FirstMax
				|| commandedOpening < -OpeningRoundingToleranceMm || commandedOpening > MaximumOpeningMm + OpeningRoundingToleranceMm
				|| world.IsActuatorClaimedByPolicy(m_GripperActuator))
			{
				error = "Gripper observation exceeds the 0–90 mm physical replay range. Keep simulated opening at or below 90 mm.";
				return false;
			}
			m_Samples.Add(new TubeReplaySample { TimeSeconds = time, JointDegrees = degrees, GripperOpeningMm = Math.Clamp(opening, 0f, MaximumOpeningMm), CommandedJointDegrees = commandedDegrees, CommandedGripperOpeningMm = Math.Clamp(commandedOpening, 0f, MaximumOpeningMm), SyncIndex = syncIndex });
			m_LastSync = syncIndex;
			return true;
		}

		public TubeReplaySample[] Snapshot() => m_Samples.ToArray();

		public static Task<string> PublishStartAsync(string directory, TubeEpisodeExport document)
		{
			return Task.Run(() => Publish(directory, document.ArtifactId + ".start.json", new { kind = "lucky.piper.episode-start", version = 1, preparedEpisode = document }));
		}

		public static Task<string> PublishFinishedAsync(string directory, TubeEpisodeExport document)
		{
			return Task.Run(() => Publish(directory, document.ArtifactId + ".json", document));
		}

		private static string Publish<T>(string directory, string filename, T value)
		{
			string fullDirectory = Path.GetFullPath(directory);
			Directory.CreateDirectory(fullDirectory);
			string destination = Path.Combine(fullDirectory, filename);
			string temporary = destination + ".partial-" + Guid.NewGuid().ToString("N");
			try
			{
				using (var stream = new FileStream(temporary, FileMode.CreateNew, FileAccess.Write, FileShare.None))
				{
					JsonSerializer.Serialize(stream, value, JsonOptions);
					stream.Flush(flushToDisk: true);
				}
				File.Move(temporary, destination, overwrite: false);
				return destination;
			}
			finally
			{
				if (File.Exists(temporary))
				{
					File.Delete(temporary);
				}
			}
		}
	}
}
