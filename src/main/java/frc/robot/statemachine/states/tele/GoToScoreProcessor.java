package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.AutoTargetUtils;
import frc.robot.util.bboard.ButtonBoard;

public class GoToScoreProcessor extends State {
    public GoToScoreProcessor(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive) {
        super(stateMachine);

        // TODO: change to processor
        startWhenActive(DriveCommands.goToPoint(drive, AutoTargetUtils::processor));
    }
}
