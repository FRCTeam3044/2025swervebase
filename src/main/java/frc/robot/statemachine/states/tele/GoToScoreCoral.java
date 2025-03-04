package frc.robot.statemachine.states.tele;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.reusable.SmartTrigger;
import frc.robot.statemachine.reusable.State;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;

public class GoToScoreCoral extends State {
        public GoToScoreCoral(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
                        Shoulder shoulder) {
                super(stateMachine);

                // Command ends when the target changes
                Command goToReef = DriveCommands.goToPoint(drive, buttonBoard::getCoralReefTarget)
                                .until(buttonBoard::coralReefJustChanged);

                // Start command initially
                startWhenActive(goToReef);

                // When the target changed last frame, then we start the new command (else it
                // would just cancel itself).
                t(buttonBoard::coralReefJustChanged).onFalse(goToReef);

                DoubleSupplier dist = buttonBoard.getCoralReefTargetDist(drive);
                SmartTrigger staging = t(() -> dist.getAsDouble() < stateMachine.stagingThreshold.get());
                // staging.whileTrue(Commands
                // .defer(() -> elevator.stageCoral(buttonBoard.getCoralReefLevel()),
                // Set.of(elevator))
                // .withName("Elevator to CoralLevel"));
                staging.whileTrue(Commands
                                .defer(() -> shoulder.stageCoral(buttonBoard.getCoralReefLevel()), Set.of(shoulder))
                                .withName("Shoulder to CoralLevel"));
                staging.whileFalse(shoulder.idle());
                startWhenActive(elevator.idle());
        }
}
