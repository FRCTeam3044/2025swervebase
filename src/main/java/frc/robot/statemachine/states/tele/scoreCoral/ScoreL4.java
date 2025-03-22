package frc.robot.statemachine.states.tele.scoreCoral;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.reusable.State;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.AutoTargetUtils.Reef;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.bboard.ButtonBoard;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ScoreL4 extends State {
        public static ConfigurableParameter<Double> postScoreDist = new ConfigurableParameter<>(0.7,
                        "Post L4 Scoring Distances");
        private static ConfigurableParameter<Double> scoreDist = new ConfigurableParameter<>(0.7,
                        "L4 Shoot Out distance");

        public ScoreL4(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, EndEffector endEffector,
                        Elevator elevator, Shoulder shoulder, LEDs LEDs) {
                super(stateMachine);

                Supplier<Pose2d> farTarget = () -> buttonBoard.getCoralReefLocation().data().poseFacing(
                                postScoreDist.get(),
                                Reef.flipped.get());

                DoubleSupplier distToRef = buttonBoard.getCoralReefReferenceDist(drive);

                Command alignElevatorAndShoulder = elevator
                                .toCoral(() -> CoralLevel.L4, distToRef)
                                .alongWith(shoulder.scoreCoral(() -> CoralLevel.L4, distToRef, () -> false));

                Command far = DriveCommands.pointControlSlow(drive, farTarget, () -> true, () -> true)
                                .until(() -> DriveCommands.pointControllerLooseConverged);
                Command close = Commands
                                .waitUntil(() -> shoulder.isAtCoralTargetFast(() -> CoralLevel.L4, distToRef)
                                                && elevator.isAtTarget())
                                .andThen(DriveCommands
                                                .pointControlSlow(drive, buttonBoard::getCoralReefTarget, () -> true,
                                                                () -> true)
                                                .until(() -> DriveCommands.pointControllerConverged).withTimeout(1.2));

                startWhenActive(Commands.sequence(far,
                                close.alongWith(Commands
                                                .waitUntil(() -> distToRef.getAsDouble() < scoreDist.get()
                                                                && elevator.isAtTarget()
                                                                && shoulder.isAtCoralTargetFast(() -> CoralLevel.L4,
                                                                                distToRef))
                                                .andThen(endEffector.algaeOut().asProxy())))

                                .withName("l4 Scoring Sequence"));

                t(endEffector::noGamePiece)
                                .whileTrue(Commands.deadline(Commands.waitSeconds(0.3), elevator
                                                .toCoral(() -> CoralLevel.L4, distToRef)
                                                .alongWith(shoulder.scoreCoral(() -> CoralLevel.L4, distToRef,
                                                                () -> false)))
                                                .andThen(Commands.parallel(elevator.idle(), shoulder.idle())));
                startWhenActive(alignElevatorAndShoulder);

                // BooleanSupplier staging = () -> !elevator.isAtTarget();
                // startWhenActive(elevator.toCoral(buttonBoard::getCoralReefLevel,
                // distanceToRef));
                // startWhenActive(shoulder.scoreCoral(buttonBoard::getCoralReefLevel,
                // distanceToRef, staging));

                // DoubleSupplier distanceToTarget = buttonBoard.getCoralReefTargetDist(drive);
                // BooleanSupplier readyToScore = () -> {
                // return distanceToTarget.getAsDouble() < stateMachine.alignmentThreshold
                // .get() && elevator.isAtTarget()
                // && shoulder.isAtCoralTarget(buttonBoard::getCoralReefLevel, distanceToRef);
                // };
                // t(readyToScore).onTrue(endEffector.coralOut());
        }
}
