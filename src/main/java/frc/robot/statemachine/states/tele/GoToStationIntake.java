package frc.robot.statemachine.states.tele;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;

public class GoToStationIntake extends State {
    public GoToStationIntake(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
            Shoulder shoulder, LEDs LEDs) {
        super(stateMachine);

        Command goToIntake = DriveCommands.goToPoint(drive, buttonBoard::getIntakeStationTarget)
                .until(buttonBoard::intakeJustChanged);
        startWhenActive(goToIntake);
        t(buttonBoard::intakeJustChanged).onFalse(goToIntake);
        startWhenActive(LEDs.goingToCoralIntake());
        startWhenActive(elevator.intakeCoral(buttonBoard.getIntakeStationTargetDist(drive)));
        startWhenActive(shoulder.intakeCoral(buttonBoard.getIntakeStationTargetDist(drive)));
    }
}
