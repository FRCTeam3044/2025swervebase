package frc.robot.statemachine.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceUtil;

public class ManualState extends State {
        public ManualState(StateMachineBase stateMachine, CommandXboxController driverController,
                        CommandXboxController operatorController, Drive drive) {
                super(stateMachine);

                DoubleSupplier xVel;
                DoubleSupplier yVel;
                DoubleSupplier rotVel;
                if (Constants.currentMode == Mode.SIM) {
                        xVel = () -> -driverController.getLeftX();
                        yVel = () -> driverController.getLeftY();
                        rotVel = () -> -driverController.getRightX();
                } else {
                        if (AllianceUtil.getAlliance() == AllianceUtil.AllianceColor.RED) {
                                xVel = () -> driverController.getLeftY();
                                yVel = () -> -driverController.getLeftX();
                                rotVel = () -> driverController.getRightX();
                        } else {
                                xVel = () -> -driverController.getLeftY();
                                yVel = () -> -driverController.getLeftX();
                                rotVel = () -> -driverController.getRightX();
                        }
                }

                startWhenActive(DriveCommands.joystickDrive(drive, xVel, yVel, rotVel).withName("Manual Drive"));

                // Lock to 0 when A button is held
                driverController
                                .a()
                                .whileTrue(DriveCommands.joystickDriveAtAngle(
                                                drive,
                                                () -> -driverController.getLeftY(),
                                                () -> -driverController.getLeftX(),
                                                () -> new Rotation2d()));

                // Switch to X pattern when X button is pressed
                driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        }
}
