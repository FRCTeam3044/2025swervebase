package frc.robot.statemachine.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
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

        public TestState(StateMachineBase stateMachine, CommandXboxController controllerOne,
                        CommandXboxController controllerTwo, Elevator elevator,
                        Shoulder shoulder, EndEffector endEffector, Climber climber, LEDs LEDs, Drive drive) {
                super(stateMachine);
                SmartXboxController testControllerOne = new SmartXboxController(controllerOne, loop);
                SmartXboxController testControllerTwo = new SmartXboxController(controllerTwo, loop);

                DoubleSupplier rightXOne = () -> -MathUtil.applyDeadband(controllerOne.getRightX(), 0.1);
                DoubleSupplier leftYOne = () -> -MathUtil.applyDeadband(controllerOne.getLeftY(), 0.1);
                DoubleSupplier leftXOne = () -> -MathUtil.applyDeadband(controllerOne.getLeftX(), 0.1);

                DoubleSupplier rightYTwo = () -> -MathUtil.applyDeadband(controllerTwo.getRightY(), 0.1);
                DoubleSupplier leftYTwo = () -> -MathUtil.applyDeadband(controllerTwo.getLeftY(), 0.1);

                // startWhenActive(LEDs.simMorseCode());
                startWhenActive(elevator.move(() -> rightYTwo.getAsDouble() * testElevatorSpeed.get()));
                startWhenActive(shoulder.manualPivot(() -> leftYTwo.getAsDouble() * testShoulderSpeed.get()));
                testControllerOne.a().or(testControllerTwo.a()).whileTrue(endEffector.algaeIn());
                testControllerOne.b().or(testControllerTwo.b()).whileTrue(endEffector.algaeOut());
                testControllerOne.x().or(testControllerTwo.x()).whileTrue(elevator.toPosition(testElevatorHeight::get));
                testControllerOne.x().or(testControllerTwo.x())
                                .runWhileFalse(elevator.move(() -> rightYTwo.getAsDouble() * testElevatorSpeed.get()));
                testControllerOne.y().or(testControllerTwo.y()).whileTrue(shoulder.toPosition(testShoulderAngle::get));
                testControllerOne.y().or(testControllerTwo.y()).runWhileFalse(
                                shoulder.manualPivot(() -> leftYTwo.getAsDouble() * testShoulderSpeed.get()));
                testControllerOne.leftBumper().or(testControllerTwo.leftBumper())
                                .onTrue(Commands.runOnce(elevator::zero));
                testControllerOne.povDown().or(testControllerTwo.povDown())
                                .whileTrue(climber.move(testClimberSpeed::get));
                testControllerOne.povUp().or(testControllerTwo.povUp())
                                .whileTrue(climber.move(() -> -testClimberSpeed.get()));
                testControllerOne.povLeft().or(testControllerTwo.povLeft()).whileTrue(climber.servoForward());
                testControllerOne.povRight().or(testControllerTwo.povRight()).whileTrue(climber.servoBackward());

                Command joystickDrive = DriveCommands.joystickDriveRobotRel(drive, leftYOne, leftXOne, rightXOne);
                startWhenActive(joystickDrive);
        }
}
