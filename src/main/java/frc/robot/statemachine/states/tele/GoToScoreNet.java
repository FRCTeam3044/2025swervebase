package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.bboard.ButtonBoard;

public class GoToScoreNet extends State {
    public GoToScoreNet(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive) {
        super(stateMachine);

        // TODO: Net
        // startWhenActive(DriveCommands.goToPoint(drive,
        // buttonBoard::getAlgaeReefTarget));
    }
}
