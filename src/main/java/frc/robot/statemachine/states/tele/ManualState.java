package frc.robot.statemachine.states.tele;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.statemachine.reusable.SmartXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.AllianceUtil;

public class ManualState extends State {
        @SuppressWarnings("unused")
        public ManualState(StateMachineBase stateMachine, CommandXboxController driver, CommandXboxController operator,
                        Drive drive, LEDs LEDs) {
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

                startWhenActive(DriveCommands.joystickDrive(drive, xVel, yVel, rotVel));
                startWhenActive(LEDs.Default());

                // Switch to X pattern when X button is pressed
                driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

                driverController.a().onTrue(
                                DriveCommands.goToPointJoystickRot(drive, new Pose2d(3, 3, new Rotation2d()), rotVel));

                
        }
}
