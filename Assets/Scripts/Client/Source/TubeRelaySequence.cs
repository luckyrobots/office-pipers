using System;

namespace OfficePipers
{
    public enum TubeRelayStage { Initial, Preparing, Homing, Opening, Outbound, Returning, Finalizing, WaitingForFollower, Finished }
    public enum TubeRelayCommand { None, Prepare, HomeBoth, BeginEpisode, StartOutbound, StartReturn, FinishSuccessful, FinishUnsuccessful }

    // One observation of the shared episode and its two independent arm drivers.
    // Object/contact validity remains the task/driver's responsibility; the sequence
    // only releases the next action after the previous action's completion evidence.
    public readonly record struct TubeRelayObservation(
        TubeEpisodeStage TaskStage,
        bool BothHome,
        bool OutboundComplete,
        bool OutboundSuccessful,
        bool ReturnComplete,
        bool ReturnSuccessful,
        bool EpisodeSuccessful,
        bool FollowerCanAdvance = true,
        bool FollowerStopped = false,
        bool RecordingCanContinue = true);

    public sealed class TubeRelaySequence
    {
        public TubeRelayStage Stage { get; private set; }
        public int CompletedCycles { get; private set; }
        public string Status { get; private set; } = "Preparing P3 → centre → P3 in one episode.";
        private bool m_CycleSuccessful;
        private readonly bool m_SingleArm;

        public TubeRelaySequence(bool singleArm = false) { m_SingleArm = singleArm; }

        public void Stop(string reason)
        {
            Stage = TubeRelayStage.Finished;
            Status = reason;
        }

        public TubeRelayCommand Advance(TubeRelayObservation state, bool continuous = true, int maximumCycles = 0)
        {
            if (Stage == TubeRelayStage.Finished) return TubeRelayCommand.None;
            if (maximumCycles < 0) return Fail("Maximum cycles must be zero (unlimited) or positive.");
            if (state.TaskStage == TubeEpisodeStage.Failed) return Fail("The shared episode failed. No new object start was prepared.");
            if (state.FollowerStopped) return Fail("A physical follower stopped. The relay will not start another action.");

            switch (Stage)
            {
                case TubeRelayStage.Initial:
                    if (state.TaskStage == TubeEpisodeStage.WaitingForPlay) return TubeRelayCommand.None;
                    if (state.TaskStage != TubeEpisodeStage.Idle) return Fail("The relay requires an unused episode at Play start.");
                    Stage = TubeRelayStage.Preparing;
                    Status = "Preparing the tube at P3; the same object will return there.";
                    return TubeRelayCommand.Prepare;

                case TubeRelayStage.Preparing:
                    if (state.TaskStage != TubeEpisodeStage.Ready) return TubeRelayCommand.None;
                    if (!state.BothHome)
                    {
                        Stage = TubeRelayStage.Homing;
                        Status = m_SingleArm ? "Returning Piper 1 to zero before opening the episode."
                            : "Returning both simulated arms to zero before opening the episode.";
                        return TubeRelayCommand.HomeBoth;
                    }
                    return BeginEpisode();

                case TubeRelayStage.Homing:
                    if (state.TaskStage != TubeEpisodeStage.Ready) return Fail(m_SingleArm
                        ? "The prepared episode changed while Piper 1 was homing." : "The prepared episode changed while both arms were homing.");
                    return state.BothHome ? BeginEpisode() : TubeRelayCommand.None;

                case TubeRelayStage.Opening:
                    if (state.TaskStage != TubeEpisodeStage.Active) return TubeRelayCommand.None;
                    Stage = TubeRelayStage.Outbound;
                    Status = "Piper 1: pick up at P3, place at the centre, then return to zero.";
                    return TubeRelayCommand.StartOutbound;

                case TubeRelayStage.Outbound:
                    if (state.TaskStage != TubeEpisodeStage.Active) return Fail("The shared episode ended before the outbound arm finished.");
                    if (!state.OutboundComplete || !state.BothHome) return TubeRelayCommand.None;
                    if (!state.OutboundSuccessful) return Finalize(false);
                    Stage = TubeRelayStage.Returning;
                    Status = (m_SingleArm ? "Piper 1" : "Opposite Piper") + ": pick up at the centre, place back at P3, then return to zero.";
                    return TubeRelayCommand.StartReturn;

                case TubeRelayStage.Returning:
                    if (state.TaskStage != TubeEpisodeStage.Active) return Fail("The shared episode ended before the return arm finished.");
                    if (!state.ReturnComplete || !state.BothHome) return TubeRelayCommand.None;
                    return Finalize(state.ReturnSuccessful);

                case TubeRelayStage.Finalizing:
                    if (state.TaskStage != TubeEpisodeStage.Complete) return TubeRelayCommand.None;
                    if (!m_CycleSuccessful || !state.EpisodeSuccessful) return Fail(m_SingleArm
                        ? "The round trip was unsuccessful. Piper 1 is home; no next cycle will start."
                        : "The round trip was unsuccessful. Both arms are home; no next cycle will start.");
                    CompletedCycles++;
                    Stage = TubeRelayStage.WaitingForFollower;
                    goto case TubeRelayStage.WaitingForFollower;

                case TubeRelayStage.WaitingForFollower:
                    if (state.TaskStage != TubeEpisodeStage.Complete) return Fail("The completed episode changed while waiting for its followers.");
                    if (!state.FollowerCanAdvance)
                    {
                        Status = "Round trip complete. Waiting for all physical followers before the next episode.";
                        return TubeRelayCommand.None;
                    }
                    if (!continuous || maximumCycles > 0 && CompletedCycles >= maximumCycles)
                        return Fail($"Completed {CompletedCycles} round trip(s). " + (m_SingleArm ? "Piper 1 is at zero." : "Both arms are at zero."));
                    if (!state.RecordingCanContinue) return Fail("Recording ended. No next relay episode was started.");
                    Stage = TubeRelayStage.Preparing;
                    Status = "Tube is back at P3. Preparing the next round-trip episode.";
                    return TubeRelayCommand.Prepare;
            }
            return TubeRelayCommand.None;
        }

        private TubeRelayCommand BeginEpisode()
        {
            Stage = TubeRelayStage.Opening;
            Status = (m_SingleArm ? "Piper 1 is at zero." : "Both arms are at zero.") + " Opening one episode for the complete round trip.";
            return TubeRelayCommand.BeginEpisode;
        }

        private TubeRelayCommand Finalize(bool successful)
        {
            m_CycleSuccessful = successful;
            Stage = TubeRelayStage.Finalizing;
            Status = successful ? (m_SingleArm ? "Tube returned to P3 and Piper 1 is at zero." : "Tube returned to P3 and both arms at zero.") +
                " Finishing the shared episode." : "A grasp or placement failed. " +
                (m_SingleArm ? "Piper 1 is home" : "Both arms are home") + "; finishing this trial without starting another.";
            return successful ? TubeRelayCommand.FinishSuccessful : TubeRelayCommand.FinishUnsuccessful;
        }

        private TubeRelayCommand Fail(string reason)
        {
            Stop(reason);
            return TubeRelayCommand.None;
        }
    }
}
