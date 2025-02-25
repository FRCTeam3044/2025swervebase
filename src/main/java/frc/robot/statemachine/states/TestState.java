package frc.robot.statemachine.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import me.nabdev.oxconfig.ConfigurableParameter;

public class TestState extends State {
        private static ConfigurableParameter<Double> testElevatorHeight = new ConfigurableParameter<>(1.0,
                        "Test Elevator Height");
        private static ConfigurableParameter<Double> testShoulderAngle = new ConfigurableParameter<>(0.0,
                        "Test Shoulder Angle");

        public TestState(StateMachineBase stateMachine, CommandXboxController controller, Elevator elevator,
                        Shoulder shoulder, EndEffector endEffector, LEDs LEDs) {
                super(stateMachine);
                SmartXboxController testController = new SmartXboxController(controller, loop);

                DoubleSupplier rightY = () -> -MathUtil.applyDeadband(controller.getRightY(), 0.1);
                DoubleSupplier leftY = () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.1);

                startWhenActive(LEDs.simMorseCode());
                startWhenActive(elevator.move(rightY));
                startWhenActive(shoulder.manualPivot(leftY));
                testController.a().whileTrue(endEffector.runIntake());
                testController.b().whileTrue(endEffector.runIntakeReverse());
                testController.x().whileTrue(elevator.toPosition(testElevatorHeight::get));
                testController.y().whileTrue(shoulder.toPosition(testShoulderAngle::get));
        }
}
