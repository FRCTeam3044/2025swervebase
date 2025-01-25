package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.ButtonBoardUtil;

public class ScoreCoral extends State{
    public ScoreCoral(StateMachineBase stateMachine, ButtonBoardUtil buttonBoard, Drive drive, EndEffector endEffector) {
        super(stateMachine);

        startWhenActive(DriveCommands.pointControl(drive, buttonBoard.getSelectedReef(), null));
        startWhenActive(endEffector.runIntakeReverse());
    }
}
