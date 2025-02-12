package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.ButtonBoardUtil;

public class GoToScoreNet extends State {
    public GoToScoreNet(StateMachineBase stateMachine, ButtonBoardUtil buttonBoard, Drive drive) {
        super(stateMachine);

        // TODO: change position to net
        startWhenActive(DriveCommands.goToPointDesiredRot(drive, buttonBoard.getAlgaeReefTarget(), null));
    }
}
