using System;
using Hazel;

namespace OfficePipers
{
    // Simulation-only sequencing. The articulated arm(s) and free tube share one MuJoCo world.
    public class TubeRoundTripDemonstrator : Entity
    {
        [Group("Tube round trip")]
        public Entity? TaskEntity;
        public Entity? OutboundDriverEntity;
        public Entity? ReturnDriverEntity;
        [Tooltip("Use Piper 1 for both legs in the qualified single-arm scene. Leave the return driver unassigned.")]
        public bool SingleArmLoop;
        public bool AutoRunOnPlay = true;
        public bool ContinuousLoops = true;
        [Tooltip("0 repeats until stopped. Each episode includes P3 → centre → P3 with a return to zero after each leg.")]
        public int MaximumEpisodes;
        [Tooltip("Also record configured virtual cameras. Off keeps a fast trajectory-only preview.")]
        public bool RecordCamerasOnPlay;
        [ReadOnly] public int CompletedEpisodes;
        [ReadOnly] public string Status = "Press Play for Piper 1: P3 → centre; opposite Piper: centre → P3.";

        private TubeRelaySequence m_Sequence = new();
        private WoodFillerTask? m_Task;
        private WoodFillerDemonstrator? m_Outbound, m_Return;
        private Vector3 m_P3, m_Cross;
        private bool m_Running;
        private int m_ControlRevision;
        private float m_StageElapsed, m_InitializationElapsed;
        private TubeRelayStage m_PreviousStage;

        protected override void OnCreate()
        {
            m_Sequence = new(SingleArmLoop);
            m_Running = AutoRunOnPlay;
        }

        [Button("Run simulated round trip")]
        public void RunRoundTrip()
        {
            if (m_Sequence.Stage != TubeRelayStage.Initial)
            {
                Status = "Stop Play and start again for a fresh round-trip run.";
                return;
            }
            m_Running = true;
        }

        [Button("Stop simulated round trip")]
        public void StopRoundTrip() { Fail("Stopped by user. No next round trip will start."); }

        protected override void OnUpdate(float deltaTime)
        {
            if (!m_Running || m_Sequence.Stage == TubeRelayStage.Finished) return;
            if (!float.IsFinite(deltaTime) || deltaTime <= 0f) return;
            m_Task ??= TaskEntity?.As<WoodFillerTask>();
            m_Outbound ??= OutboundDriverEntity?.As<WoodFillerDemonstrator>();
            m_Return ??= SingleArmLoop ? m_Outbound : ReturnDriverEntity?.As<WoodFillerDemonstrator>();
            if (m_Task == null || m_Outbound == null || m_Return == null)
            {
                m_InitializationElapsed += deltaTime;
                if (m_InitializationElapsed > 3f) Fail(SingleArmLoop
                    ? "Assign the shared task and Piper 1 driver."
                    : "Assign the shared task, outbound driver and opposite-arm driver.");
                return;
            }
            bool modeMatches = SingleArmLoop
                ? m_Task.SingleArmRoundTrip && !m_Task.TwoRobotRelay && ReturnDriverEntity?.ID is not > 0
                : m_Task.TwoRobotRelay && !m_Task.SingleArmRoundTrip && !ReferenceEquals(m_Outbound, m_Return) &&
                    OutboundDriverEntity!.ID != ReturnDriverEntity!.ID && m_Return.JointPrefix == "piper3_" && m_Return.ActuatorOffset == 7;
            if (!modeMatches || m_Task.Workflow != TubeWorkflow.SimulationOnly ||
                m_Outbound.JointPrefix != "" || m_Outbound.ActuatorOffset != 0)
            {
                Fail(SingleArmLoop ? "Single-arm loop requires the qualified simulation-only Piper 1 world and no return driver."
                    : "Round trip requires the qualified simulation-only two-arm world, with separate canonical and piper3_ drivers.");
                return;
            }
            if (m_Outbound.RelayLegFaulted || m_Return.RelayLegFaulted)
            {
                Fail("A simulated arm could not finish its motion: " + (m_Outbound.RelayLegFaulted ? m_Outbound.Status : m_Return.Status));
                return;
            }
            if (m_Sequence.Stage != TubeRelayStage.Initial && m_Task.ControlRevision != m_ControlRevision)
            {
                Fail("A manual task action changed this episode. The round trip stopped without resetting the tube.");
                return;
            }
            if (m_Sequence.Stage != m_PreviousStage) { m_StageElapsed = 0f; m_PreviousStage = m_Sequence.Stage; }
            m_StageElapsed += deltaTime;
            if (m_StageElapsed > 30f && m_Sequence.Stage is TubeRelayStage.Preparing or TubeRelayStage.Homing or TubeRelayStage.Opening or TubeRelayStage.Finalizing)
            {
                Fail("The round trip timed out while " + m_Sequence.Stage + ". No new tube start was prepared.");
                return;
            }

            var observation = new TubeRelayObservation(m_Task.EpisodeStage,
                m_Outbound.HomeReady && m_Return.HomeReady,
                m_Outbound.RelayLegComplete, m_Outbound.RelayLegSuccessful,
                m_Return.RelayLegComplete, m_Return.RelayLegSuccessful,
                m_Task.EpisodeSucceeded, m_Task.FollowerCanAdvance,
                m_Task.FollowerStatus == Hazel.Data.EpisodeFollowerStatus.Stopped,
                m_Task.RecordingCanContinue);
            TubeRelayCommand command = m_Sequence.Advance(observation, ContinuousLoops, MaximumEpisodes);
            CompletedEpisodes = m_Sequence.CompletedCycles;
            Status = m_Sequence.Status;
            if (m_Sequence.Stage == TubeRelayStage.Outbound && command == TubeRelayCommand.None) Status += " " + m_Outbound.Status;
            if (m_Sequence.Stage == TubeRelayStage.Returning && command == TubeRelayCommand.None) Status += " " + m_Return.Status;

            switch (command)
            {
                case TubeRelayCommand.Prepare:
                    m_Task.RecordVirtualScene = RecordCamerasOnPlay;
                    m_Task.PrepareNextEpisode();
                    m_ControlRevision = m_Task.ControlRevision;
                    break;
                case TubeRelayCommand.HomeBoth:
                    if (!m_Outbound.HomeRelayArm() || (!SingleArmLoop && !m_Return.HomeRelayArm()))
                        Fail(SingleArmLoop ? "Could not prepare Piper 1 at zero: " + m_Outbound.Status
                            : "Could not prepare both simulated arms at zero: " + m_Outbound.Status + " " + m_Return.Status);
                    break;
                case TubeRelayCommand.BeginEpisode:
                    m_P3 = m_Task.PreparedStartCentre;
                    m_Cross = m_Task.PreparedGoalCentre;
                    m_Task.BeginSimulationEpisode();
                    m_ControlRevision = m_Task.ControlRevision;
                    break;
                case TubeRelayCommand.StartOutbound:
                    if (!m_Task.BeginRelayLeg(m_Cross, "Outbound: P3 → centre cross") ||
                        !m_Outbound.StartRelayLeg(m_Task.PreparedTubeCentre, m_Cross))
                        Fail("Could not start the outbound pickup: " + m_Outbound.Status);
                    break;
                case TubeRelayCommand.StartReturn:
                    // Pick up the same free body where the outbound leg left it.
                    // No reset/teleport and no new recording boundary occurs here.
                    if (!m_Task.BeginRelayLeg(m_P3, "Return: centre cross → P3") ||
                        !m_Return.StartRelayLeg(m_Task.PreparedTubeCentre, m_P3))
                        Fail("Could not start the return pickup: " + m_Return.Status);
                    break;
                case TubeRelayCommand.FinishSuccessful:
                    m_Task.FinishSuccessfully();
                    m_ControlRevision = m_Task.ControlRevision;
                    break;
                case TubeRelayCommand.FinishUnsuccessful:
                    m_Task.FinishUnsuccessfully();
                    m_ControlRevision = m_Task.ControlRevision;
                    break;
            }
        }

        private void Fail(string reason)
        {
            m_Running = false;
            m_Sequence.Stop(reason);
            m_Outbound?.StopDemonstration();
            if (!ReferenceEquals(m_Outbound, m_Return)) m_Return?.StopDemonstration();
            if (m_Task?.EpisodeActive == true) m_Task.FinishUnsuccessfully();
            Status = reason;
        }
    }
}
