package frc.robot.statemachine.states.tele.scoreCoral;

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
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.bboard.ButtonBoard;

public class ScoreL3 extends State {
    public ScoreL3(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, EndEffector endEffector,
            Elevator elevator, Shoulder shoulder, LEDs LEDs) {
        super(stateMachine);

        DoubleSupplier distanceToRef = buttonBoard.getCoralReefReferenceDist(drive);
        startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getCoralReefTarget));

        BooleanSupplier staging = () -> !elevator.isAtTarget();
        startWhenActive(elevator.toCoral(() -> CoralLevel.L3, distanceToRef));
        startWhenActive(shoulder.scoreCoral(() -> CoralLevel.L3, distanceToRef, staging));

        BooleanSupplier readyToScore = () -> {
            return elevator.isAtTarget()
                    && shoulder.isAtCoralTargetFast(() -> CoralLevel.L3, distanceToRef)
                    && DriveCommands.pointControllerLooseConverged;
        };
        t(readyToScore).onTrue(endEffector.coralOutSlow());
    }
}
