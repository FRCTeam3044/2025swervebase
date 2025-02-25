package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.util.AutoTargetUtils;
import frc.robot.util.bboard.ButtonBoard;

public class ScoreAlgaeProcessor extends State {
    public ScoreAlgaeProcessor(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive,
            EndEffector endEffector, LEDs LEDs) {
        super(stateMachine);

        // TODO: change to processor
        startWhenActive(DriveCommands.pointControl(drive, AutoTargetUtils::processor));
        startWhenActive(endEffector.runIntakeReverse());
        startWhenActive(LEDs.intakingAndScoringAlgae());
    }
}
