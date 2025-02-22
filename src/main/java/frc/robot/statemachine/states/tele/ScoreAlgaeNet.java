package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.bboard.ButtonBoard;

public class ScoreAlgaeNet extends State {
    public ScoreAlgaeNet(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive,
            EndEffector endEffector, LEDs LEDs) {
        super(stateMachine);

        // TODO: change to net
        startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getCoralReefTarget));
        startWhenActive(endEffector.runIntakeReverse());
        startWhenActive(LEDs.intakingAndScoringAlgae());
    }
}
