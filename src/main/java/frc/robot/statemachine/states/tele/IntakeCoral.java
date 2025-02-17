package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;

public class IntakeCoral extends State {
    public IntakeCoral(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
            Shoulder shoulder, EndEffector endEffector, LEDs LEDs) {
        super(stateMachine);

        startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getIntakeStationTarget));
        startWhenActive(endEffector.runIntake());
        startWhenActive(LEDs.intakingAndScoringCoral());
        startWhenActive(elevator.intakeCoral(buttonBoard.getIntakeStationTargetDist(drive)));
        startWhenActive(shoulder.intakeCoral(buttonBoard.getIntakeStationTargetDist(drive)));
    }
}
