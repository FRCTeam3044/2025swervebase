package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;

public class GoToReefIntake extends State {
    public GoToReefIntake(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive, LEDs LEDs,
            Shoulder shoulder, Elevator elevator) {
        super(stateMachine);

        startWhenActive(DriveCommands.goToPoint(drive, buttonBoard::getAlgaeReefFarTarget));
        startWhenActive(LEDs.goingToAlgaeIntake());
        startWhenActive(elevator.idle());
        startWhenActive(shoulder.idle());
    }
}
