package frc.robot.statemachine.states.tele;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.reusable.State;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;

public class IntakeCoral extends State {
    public IntakeCoral(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
            Shoulder shoulder, EndEffector endEffector, LEDs LEDs) {
        super(stateMachine);

        startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getIntakeStationTarget));
        startWhenActive(endEffector.coralIn());
        startWhenActive(LEDs.intakingAndScoringCoral());
        startWhenActive(elevator.intakeCoral(buttonBoard.getIntakeStationReferenceDist(drive)));
        BooleanSupplier elevatorCloseToTarget = elevator::isAtTarget;
        DoubleSupplier dist = buttonBoard.getIntakeStationTargetDist(drive);
        BooleanSupplier staging = () -> dist.getAsDouble() > stateMachine.alignmentThreshold.get()
                || !elevatorCloseToTarget.getAsBoolean();
        startWhenActive(shoulder.intakeCoral(buttonBoard.getIntakeStationReferenceDist(drive), staging));
    }
}
