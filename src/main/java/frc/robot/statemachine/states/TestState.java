package frc.robot.statemachine.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import me.nabdev.oxconfig.ConfigurableParameter;

public class TestState extends State {
        private ConfigurableParameter<Double> testElevatorHeight = new ConfigurableParameter<>(1.0,
                        "Test Elevator Height");
        private ConfigurableParameter<Double> testShoulderAngle = new ConfigurableParameter<>(0.0,
                        "Test Shoulder Angle");

        private ConfigurableParameter<Double> testShoulderSpeed = new ConfigurableParameter<>(0.2,
                        "Test Shoulder Speed");
        private ConfigurableParameter<Double> testElevatorSpeed = new ConfigurableParameter<>(0.2,
                        "Test Elevator Speed");

        private ConfigurableParameter<Double> testClimberSpeed = new ConfigurableParameter<>(0.2,
                        "Test Climber Speed");

        public TestState(StateMachineBase stateMachine, CommandXboxController controller, Elevator elevator,
                        Shoulder shoulder, EndEffector endEffector, Climber climber, LEDs LEDs) {
                super(stateMachine);
                SmartXboxController testController = new SmartXboxController(controller, loop);

                DoubleSupplier rightY = () -> -MathUtil.applyDeadband(controller.getRightY(), 0.1);
                DoubleSupplier leftY = () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.1);

                startWhenActive(LEDs.simMorseCode());
                startWhenActive(elevator.move(() -> rightY.getAsDouble() * testElevatorSpeed.get()));
                startWhenActive(shoulder.manualPivot(() -> leftY.getAsDouble() * testShoulderSpeed.get()));
                testController.a().whileTrue(endEffector.algaeIn());
                testController.b().whileTrue(endEffector.algaeOut());
                testController.x().whileTrue(elevator.toPosition(testElevatorHeight::get));
                testController.x().runWhileFalse(elevator.move(() -> rightY.getAsDouble() * testElevatorSpeed.get()));
                testController.y().whileTrue(shoulder.toPosition(testShoulderAngle::get));
                testController.y().runWhileFalse(
                                shoulder.manualPivot(() -> leftY.getAsDouble() * testShoulderSpeed.get()));
                testController.leftBumper().onTrue(Commands.runOnce(elevator::zero));
                testController.povDown().whileTrue(climber.move(testClimberSpeed::get));
                testController.povUp().whileTrue(climber.move(() -> -testClimberSpeed.get()));
                testController.povLeft().whileTrue(climber.servoForward());
                testController.povRight().whileTrue(climber.servoBackward());
        }
}
