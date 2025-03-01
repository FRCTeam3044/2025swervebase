package frc.robot.statemachine.states.tele;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.reusable.SmartTrigger;
import frc.robot.statemachine.reusable.State;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;

public class GoToStationIntake extends State {
    public GoToStationIntake(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
            Shoulder shoulder, LEDs LEDs) {
        super(stateMachine);

        Command goToIntake = DriveCommands.goToPoint(drive, buttonBoard::getIntakeStationTarget)
                .until(buttonBoard::intakeJustChanged);
        startWhenActive(goToIntake);
        t(buttonBoard::intakeJustChanged).onFalse(goToIntake);
        startWhenActive(LEDs.goingToCoralIntake());
        DoubleSupplier dist = buttonBoard.getIntakeStationTargetDist(drive);
        SmartTrigger staging = t(() -> dist.getAsDouble() < stateMachine.stagingThreshold.get());
        // staging.whileTrue(elevator.stageIntake());
        staging.whileTrue(shoulder.stageIntake());
        staging.whileFalse(shoulder.idle());
        startWhenActive(elevator.idle());
    }
}
