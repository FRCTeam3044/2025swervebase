package frc.robot.statemachine.states.tele;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.AllianceUtil;

public class ManualState extends State {
        public ManualState(StateMachineBase stateMachine, CommandXboxController driver, CommandXboxController operator,
                        Drive drive, Elevator elevator, Shoulder shoulder, EndEffector endEffector, LEDs LEDs) {
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
                driverController.a().onTrue(
                                DriveCommands.goToPointJoystickRot(drive, new Pose2d(3, 3, new Rotation2d()), rotVel));

                operatorController.leftTrigger().onTrue(endEffector.runIntakeReverse());
                operatorController.rightTrigger().onTrue(endEffector.runIntake());

                DoubleSupplier rightY = () -> -MathUtil.applyDeadband(operatorController.getHID().getRightY(), 0.01);
                DoubleSupplier leftY = () -> -MathUtil.applyDeadband(operatorController.getHID().getLeftY(), 0.05);

                startWhenActive(elevator.elevatorMove(rightY));
                startWhenActive(shoulder.manualPivot(leftY));
        }
}
