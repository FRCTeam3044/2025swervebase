package frc.robot.statemachine.states;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;

public class TestState extends State {

        public TestState(StateMachineBase stateMachine, CommandXboxController controller) {
                super(stateMachine);
        }
}
