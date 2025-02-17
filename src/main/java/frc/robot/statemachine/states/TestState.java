package frc.robot.statemachine.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;

public class TestState extends State {

        public TestState(StateMachineBase stateMachine, CommandXboxController controller, Elevator elevator,
                        Shoulder shoulder, EndEffector endEffector, LEDs LEDs) {
                super(stateMachine);
                SmartXboxController testController = new SmartXboxController(controller, loop);

                DoubleSupplier rightY = () -> -MathUtil.applyDeadband(controller.getRightY(), 0.1);
                DoubleSupplier leftY = () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.1);

                startWhenActive(LEDs.simMorseCode());
                startWhenActive(elevator.elevatorMove(rightY));
                startWhenActive(shoulder.manualPivot(leftY));
                testController.a().onTrue(endEffector.runIntake());
                testController.b().onTrue(endEffector.runIntakeReverse());
        }
}
