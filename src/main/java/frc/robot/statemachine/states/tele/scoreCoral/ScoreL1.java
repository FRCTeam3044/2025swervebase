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
import frc.robot.util.bboard.ButtonBoard;

public class ScoreL1 extends State {
    public ScoreL1(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, EndEffector endEffector,
            Elevator elevator, Shoulder shoulder, LEDs LEDs) {
        super(stateMachine);

        DoubleSupplier distanceToRef = buttonBoard.getCoralReefReferenceDist(drive);
        startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getCoralReefTarget));

        BooleanSupplier staging = () -> !elevator.isAtTarget();
        startWhenActive(elevator.toCoral(buttonBoard::getCoralReefLevel, distanceToRef));
        startWhenActive(shoulder.scoreCoral(buttonBoard::getCoralReefLevel, distanceToRef, staging));

        DoubleSupplier distanceToTarget = buttonBoard.getCoralReefTargetDist(drive);
        BooleanSupplier readyToScore = () -> {
            return distanceToTarget.getAsDouble() < stateMachine.alignmentThreshold
                    .get() && elevator.isAtTarget()
                    && shoulder.isAtCoralTarget(buttonBoard::getCoralReefLevel, distanceToRef);
        };
        t(readyToScore).onTrue(endEffector.coralOut());
    }
}
