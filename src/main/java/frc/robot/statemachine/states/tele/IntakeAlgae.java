package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.util.bboard.ButtonBoard;

public class IntakeAlgae extends State {
    public IntakeAlgae(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
            EndEffector endEffector, LEDs LEDs) {
        super(stateMachine);

        startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getAlgaeReefTarget));
        startWhenActive(endEffector.runIntake());
        startWhenActive(LEDs.intakingAndScoringAlgae());
    }
}
