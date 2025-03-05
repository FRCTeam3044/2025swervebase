package frc.robot.statemachine.states.tele;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.statemachine.reusable.SmartTrigger;
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
import frc.robot.util.AllianceUtil;
import frc.robot.util.bboard.ButtonBoard;

public class ManualState extends State {
        public ManualState(StateMachineBase stateMachine, CommandXboxController driver, CommandXboxController operator,
                        Drive drive, Elevator elevator, Shoulder shoulder, EndEffector endEffector, LEDs LEDs,
                        ButtonBoard bboard, Climber climber) {
                super(stateMachine);

                SmartXboxController driverController = new SmartXboxController(driver, loop);
                SmartXboxController operatorController = new SmartXboxController(operator, loop);

                DoubleSupplier xVel;
                DoubleSupplier yVel;
                DoubleSupplier rotVel;

                if (Constants.currentMode == Mode.SIM && !Constants.drivePracticeSim) {
                        xVel = () -> driver.getLeftX();
                        yVel = () -> -driver.getLeftY();
                        rotVel = () -> -driver.getRightX();
                } else {
                        xVel = () -> (AllianceUtil.getAlliance() == AllianceUtil.AllianceColor.RED ? 1 : -1)
                                        * driver.getLeftY();
                        yVel = () -> (AllianceUtil.getAlliance() == AllianceUtil.AllianceColor.RED ? 1 : -1)
                                        * driver.getLeftX();
                        rotVel = () -> -driver.getRightX();
                }

                Command joystickDrive = DriveCommands.joystickDrive(drive, xVel, yVel, rotVel);
                startWhenActive(joystickDrive);
                startWhenActive(LEDs.Default());

                // Switch to X pattern when X button is pressed
                driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive).withName("X mode"));
                driverController.x().onFalse(joystickDrive);

                operatorController.leftTrigger().whileTrue(endEffector.algaeIn());
                operatorController.rightTrigger().whileTrue(endEffector.algaeOut());

                t(bboard::climbUp).whileTrue(climber.up());
                t(bboard::climbDown).whileTrue(climber.down());

                DoubleSupplier rightY = () -> -MathUtil.applyDeadband(operatorController.getHID().getRightY(), 0.1);
                DoubleSupplier leftY = () -> -MathUtil.applyDeadband(operatorController.getHID().getLeftY(), 0.1);

                SmartTrigger manualElevator = t(() -> Math.abs(rightY.getAsDouble()) > 0.1 && bboard.fullManual());
                SmartTrigger manualShoulder = t(() -> Math.abs(leftY.getAsDouble()) > 0.1 && bboard.fullManual());

                manualElevator.whileTrue(elevator.move(rightY));
                manualShoulder.whileTrue(shoulder.manualPivot(leftY));
                manualElevator.whileFalse(elevator.idle());

                SmartTrigger semiAuto = t(bboard::semiAuto);
                SmartTrigger idle = t(bboard::idleInManual);
                SmartTrigger intake = t(bboard::intakeInManual);

                semiAuto.and(idle).whileTrue(elevator.idle().alongWith(shoulder.idle()));
                semiAuto.and(intake).whileTrue(elevator.intakeCoral().alongWith(shoulder.intakeCoral()));
                semiAuto.and(idle.or((intake)).negate()).whileTrue(elevator.toCoral(bboard::getCoralReefLevel)
                                .alongWith(shoulder.scoreCoral(bboard::getCoralReefLevel)));
        }
}
