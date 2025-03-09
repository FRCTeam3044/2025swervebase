package frc.robot.statemachine.states.tele.scoreCoral;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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

        Supplier<Pose2d> farTarget = () -> {
            return buttonBoard.getCoralReefLocation().data().poseFacing(postScoreDist.get(),
                    Reef.flipped.get());

        };
        startWhenActive(Commands.run(() -> Logger.recordOutput("L4 Far Target", farTarget.get())));

        DoubleSupplier distToRef = buttonBoard.getCoralReefReferenceDist(drive);

        Command alignElevatorAndShoulder = elevator
                .toCoral(() -> CoralLevel.L4, distToRef)
                .alongWith(shoulder.scoreCoral(() -> CoralLevel.L4, distToRef, () -> false));

        Command far = DriveCommands.pointControlSlow(drive, farTarget, () -> false, () -> true)
                .until(() -> DriveCommands.pointControllerConverged);
        Command close = Commands
                .waitUntil(() -> shoulder.isAtCoralTarget(() -> CoralLevel.L4, distToRef) && elevator.isAtTarget())
                .andThen(DriveCommands.pointControlSlow(drive, buttonBoard::getCoralReefTarget, () -> true, () -> true)
                        .until(() -> DriveCommands.pointControllerConverged).withTimeout(1.2));
        startWhenActive(Commands.sequence(far, Commands.sequence(close, shoulder.idle())
                .alongWith(
                        Commands.waitUntil(() -> distToRef.getAsDouble() < scoreDist.get())
                                .andThen(endEffector.algaeOut()))));

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
