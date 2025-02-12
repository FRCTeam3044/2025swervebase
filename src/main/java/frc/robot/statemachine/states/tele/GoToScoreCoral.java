package frc.robot.statemachine.states.tele;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.ButtonBoardUtil;

public class GoToScoreCoral extends State {
    public GoToScoreCoral(StateMachineBase stateMachine, ButtonBoardUtil buttonBoard, Drive drive) {
        super(stateMachine);

        // TODO: change to 1-6 sides of reef
        startWhenActive(Commands
                .deferredProxy(() -> DriveCommands.goToPointDesiredRot(drive, buttonBoard.getCoralReefTarget(),
                        buttonBoard.getCoralReefTarget()::getRotation)));
    }
}
