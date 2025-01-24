package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.ButtonBoardUtil;

public class IntakeCoral extends State {
    public IntakeCoral(StateMachineBase stateMachine, ButtonBoardUtil buttonBoard, Drive drive, Elevator elevator, EndEffector endEffector) {
        super(stateMachine);

        // TODO: Get others to make heights and (Marcus) rotation
        startWhenActive(DriveCommands.pointControl(drive, buttonBoard.getSelectedStationPose(), null));
        startWhenActive(endEffector.runIntake());
    }
}
