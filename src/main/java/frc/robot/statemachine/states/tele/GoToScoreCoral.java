package frc.robot.statemachine.states.tele;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;

public class GoToScoreCoral extends State {
    public GoToScoreCoral(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
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
        startWhenActive(Commands
                .defer(() -> elevator.toCoral(buttonBoard.getCoralReefLevel(),
                        buttonBoard.getCoralReefTargetDist(drive)), Set.of(elevator))
                .withName("Elevator to CoralLevel"));
        startWhenActive(Commands
                .defer(() -> shoulder.scoreCoral(buttonBoard.getCoralReefLevel(),
                        buttonBoard.getCoralReefTargetDist(drive)), Set.of(shoulder))
                .withName("Shoulder to CoralLevel"));
    }
}
