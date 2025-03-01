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

public class IntakeAlgae extends State {
    public IntakeAlgae(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
            Shoulder shoulder,
            EndEffector endEffector, LEDs LEDs) {
        super(stateMachine);

        startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getAlgaeReefTarget));
        startWhenActive(endEffector.runIntake());
        BooleanSupplier elevatorCloseToTarget = elevator::isAtTarget;
        DoubleSupplier dist = buttonBoard.getAlgaeReefTargetDist(drive);
        BooleanSupplier staging = () -> dist.getAsDouble() > stateMachine.alignmentThreshold.get()
                || !elevatorCloseToTarget.getAsBoolean();
        startWhenActive(
                elevator.algaeIntake(buttonBoard::getAlgaeReefLocation, buttonBoard.getAlgaeReefReferenceDist(drive)));
        startWhenActive(shoulder.algaeIntake(buttonBoard::getAlgaeReefLocation, dist, staging));
        startWhenActive(LEDs.intakingAndScoringAlgae());
    }
}
