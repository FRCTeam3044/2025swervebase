package frc.robot.statemachine.states;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;

public class TestState extends State {

        public TestState(StateMachineBase stateMachine, CommandXboxController controller, Elevator elevator, Shoulder shoulder, EndEffector endEffector) {
                super(stateMachine);
                @SuppressWarnings("unused")
                SmartXboxController testController = new SmartXboxController(controller, loop);

                startWhenActive(elevator.elevatorMove(controller::getRightY));
                startWhenActive(shoulder.manualPivot(controller::getLeftY));
                controller.a().onTrue(endEffector.runIntake());
                controller.b().onTrue(endEffector.runIntakeReverse());
        }
}
