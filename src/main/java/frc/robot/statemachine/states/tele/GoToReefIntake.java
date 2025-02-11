package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.ButtonBoardUtil;

public class GoToReefIntake extends State {
    public GoToReefIntake(StateMachineBase stateMachine, ButtonBoardUtil buttonBoard, Drive drive, LEDs LEDs) {
        super(stateMachine);

        // TODO: rotation
        startWhenActive(DriveCommands.goToPointDesiredRot(drive, buttonBoard.getAlgaeReefTarget(), null));
        startWhenActive(LEDs.goingToAlgaeIntake());
    }
}
