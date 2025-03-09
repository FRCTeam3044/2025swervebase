package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.AutoTargetUtils;
import frc.robot.util.bboard.ButtonBoard;

public class ScoreAlgaeProcessor extends State {
    public ScoreAlgaeProcessor(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive,
            EndEffector endEffector, LEDs LEDs, Elevator elevator, Shoulder shoulder) {
        super(stateMachine);

        startWhenActive(elevator.intakeCoral());
        startWhenActive(shoulder.processor());
        startWhenActive(DriveCommands.pointControl(drive, AutoTargetUtils::processor));
        t(() -> shoulder.isAtProcessorTarget() && elevator.isAtTarget() && DriveCommands.pointControllerConverged)
                .onTrue(endEffector.algaeOut());
        startWhenActive(LEDs.intakingAndScoringAlgae());

    }
}
