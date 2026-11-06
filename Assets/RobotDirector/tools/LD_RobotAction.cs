using System;
using Hazel;

namespace CanTake
{
	/// <summary>
	/// One step in a RobotDirector's sequence. Move eases a hand to the Target entity, Set Input
	/// writes any one of the graph's inputs, and Wait just pauses. After the step the director
	/// holds for Hold seconds (plus the move's Duration, for a Move) before starting the next.
	/// </summary>
	public class RobotAction : Entity
	{
		private const int MoveType = 0;
		private const int SetInputType = 1;
		private const int WaitType = 2;

		[Dropdown("Move", "Set Input", "Wait")]
		[Tooltip("Move eases a hand to the Target. Set Input writes one graph input. Wait only pauses.")]
		public int Type = MoveType;

		[Units("s")]
		[Slider(0.0f, 10.0f)]
		[Tooltip("Time to wait after this step before the next one starts. A Move also waits for its Duration first.")]
		public float Hold = 0.0f;

		[Group("Move")]
		[Dropdown("Left Arm", "Right Arm")]
		[Tooltip("Which hand moves. The other keeps its last target. Ignored on a single-arm robot (Panda).")]
		public int Arm = 0;

		[Group("Move")]
		[Tooltip("The hand moves to this entity's world position.")]
		public Entity? Target;

		[Group("Move")]
		[Units("s")]
		[Slider(0.1f, 10.0f)]
		[Tooltip("Time to ease from the current pose to the target.")]
		public float Duration = 2.0f;

		[Group("Move")]
		[ToggleButton("Solve Orientation", "Position Only")]
		[Tooltip("Also hold the hand at Orientation. Position Only leaves the wrist wherever the position solve puts it.")]
		public bool SolveOrientation = false;

		[Group("Move")]
		[Units("deg")]
		[EditCondition("SolveOrientation")]
		[Tooltip("Hand orientation in world XYZ degrees, used with Solve Orientation.")]
		public Vector3 Orientation = Vector3.Zero;

		// Must list the RobotInput members in order: the selected index is cast straight to it
		[Group("Set Input")]
		[Dropdown("Start", "Duration",
			"LeftArm_Target_Position", "LeftArm_Target_Orientation", "LeftArm_Solve_Orientation",
			"LeftArm_Pole_Target", "LeftArm_Pole_Weight", "LeftArm_High_Accuracy", "LeftArm_Tracking_Mode",
			"RightArm_Target_Position", "RightArm_Target_Orientation", "RightArm_Solve_Orientation",
			"RightArm_Pole_Target", "RightArm_Pole_Weight", "RightArm_High_Accuracy", "RightArm_Tracking_Mode")]
		[Tooltip("The graph input to write. Only the value field matching its type is used. On a single-arm robot (Panda), LeftArm_ and RightArm_ inputs both write the unprefixed input.")]
		public int Input = 0;

		[Group("Set Input")]
		[Tooltip("Value for a Bool input.")]
		public bool BoolValue = false;

		[Group("Set Input")]
		[Tooltip("Value for a Float input.")]
		public float FloatValue = 0.0f;

		[Group("Set Input")]
		[Tooltip("Value for a Vector3 input. Ignored when Vector Source is set.")]
		public Vector3 VectorValue = Vector3.Zero;

		[Group("Set Input")]
		[Tooltip("Optional: use this entity's world position as the Vector3 value.")]
		public Entity? VectorSource;

		public RobotInput SelectedInput => (RobotInput)Input;

		/// <summary>Runs the step and returns how long the director should wait before the next.</summary>
		public float Run(RobotControllerComponent controller, bool singleArm = false)
		{
			switch (Type)
			{
				case MoveType:
					if (Target is null)
					{
						Log.Warn($"[RobotAction] Move on {Tag} has no Target; skipping it.");
						return Hold;
					}
					controller.SetInputBool(RobotInput.LeftArm_Solve_Orientation.ForArm(Arm, singleArm), SolveOrientation);
					controller.SetInputVector3(RobotInput.LeftArm_Target_Orientation.ForArm(Arm, singleArm), Orientation);
					controller.SetInputVector3(RobotInput.LeftArm_Target_Position.ForArm(Arm, singleArm), Target.Transform.WorldTranslation);
					controller.SetInputFloat(RobotInput.Duration.ID(), Duration);
					controller.SetInputTrigger(RobotInput.Start.ID());
					return Duration + Hold;

				case SetInputType:
					RobotInput input = SelectedInput;
					Identifier id = input.ID(singleArm);
					switch (input.Kind())
					{
						case RobotInputKind.Trigger: controller.SetInputTrigger(id); break;
						case RobotInputKind.Bool: controller.SetInputBool(id, BoolValue); break;
						case RobotInputKind.Float: controller.SetInputFloat(id, FloatValue); break;
						case RobotInputKind.Vector3:
							controller.SetInputVector3(id, VectorSource?.Transform.WorldTranslation ?? VectorValue);
							break;
					}
					return Hold;

				default:
					return Hold;
			}
		}
	}
}
