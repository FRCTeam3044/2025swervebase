package frc.robot.statemachine.states.tele.scoreCoral;

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
import me.nabdev.oxconfig.ConfigurableParameter;

public class ScoreL4Auto extends State {
    private static ConfigurableParameter<Double> scoreDist = new ConfigurableParameter<>(0.6,
            "L4 Shoot Out distance (auto)");

    public ScoreL4Auto(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, EndEffector endEffector,
            Elevator elevator, Shoulder shoulder, LEDs LEDs) {
        super(stateMachine);

        // Supplier<Pose2d> farTarget = () ->
        // buttonBoard.getCoralReefLocation().data().poseFacing(
        // postScoreDist.get(),
        // Reef.flipped.get());

        DoubleSupplier distToRef = buttonBoard.getCoralReefReferenceDist(drive);

        // Command alignElevatorAndShoulder = elevator
        // .toCoral(() -> CoralLevel.L4, distToRef)
        // .alongWith(shoulder.scoreCoral(() -> CoralLevel.L4, distToRef, () -> false));

        // Command far = DriveCommands.pointControlSlow(drive, farTarget, () -> true, ()
        // -> true)
        // .until(() -> DriveCommands.pointControllerLooseConverged);
        // Command close = Commands
        // .waitUntil(() -> shoulder.isAtCoralTargetFast(() -> CoralLevel.L4, distToRef)
        // && elevator.isAtTarget())
        // .andThen(
        // .until(() -> DriveCommands.pointControllerConverged).withTimeout(1.2));
        startWhenActive(DriveCommands
                .pointControlSlow(drive, buttonBoard::getCoralReefTarget, () -> true,
                        () -> true));
        t(() -> distToRef.getAsDouble() < scoreDist.get()
                && shoulder.isAtCoralTargetFast(() -> CoralLevel.L4, distToRef) && elevator.isAtTarget())
                .onTrue(endEffector.coralOut());

        // startWhenActive(Commands.sequence(far,
        // close.alongWith(Commands
        // .waitUntil(() -> distToRef.getAsDouble() < scoreDist.get()
        // && elevator.isAtTarget()
        // && shoulder.isAtCoralTargetFast(() -> CoralLevel.L4,
        // distToRef))
        // .andThen(endEffector.algaeOut().asProxy())))

        // .withName("l4 Scoring Sequence"));

        // t(endEffector::noGamePiece)
        // .whileTrue(Commands.deadline(Commands.waitSeconds(0.3), elevator
        // .toCoral(() -> CoralLevel.L4, distToRef)
        // .alongWith(shoulder.scoreCoral(() -> CoralLevel.L4, distToRef,
        // () -> false)))
        // .andThen(Commands.parallel(elevator.idle(), shoulder.idle())));
        // startWhenActive(alignElevatorAndShoulder);
    }
}
