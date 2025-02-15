package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.bboard.ButtonBoard;

public class GoToScoreCoral extends State {
    public GoToScoreCoral(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive) {
        super(stateMachine);

        startWhenActive(DriveCommands.goToPoint(drive, buttonBoard::getCoralReefTarget));
        // TODO: Elevator and Shoulder?
    }
}
