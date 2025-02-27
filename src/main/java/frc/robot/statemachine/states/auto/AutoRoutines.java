package frc.robot.statemachine.states.auto;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;

public class AutoRoutines {
    private Drive drive;
    private Elevator elevator;
    private EndEffector endEffector;

    public AutoRoutines(Drive drive, Elevator elevator, EndEffector endEffector) {
        this.drive = drive;
        this.elevator = elevator;
        this.endEffector = endEffector;
    }

    public AutoRoutine testAuto() {
        AutoRoutine routine = RobotContainer.getInstance().autoFactory.newRoutine("Test");

        // Load the routine's trajectories
        AutoTrajectory example = routine.trajectory("Testy Testy", 0);
        Command outtake = Commands.waitUntil(elevator::isAtTarget)
                .andThen(endEffector.runIntakeReverse().until(() -> !endEffector.hasCoral()))
                .andThen(elevator.idle())
                .withName("Intake when elevator ready");

        RobotContainer.getInstance().startPose = example.getInitialPose().get();
        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
                Commands.sequence(
                        example.resetOdometry(),
                        example.cmd(),
                        Commands.runOnce(drive::stop)));

        example.atTime("elevator").onTrue(
                elevator.toCoral(() -> CoralLevel.L2, distToEnd(example))
                        .until(() -> !endEffector.hasCoral())
                        .andThen(elevator.idle()));
        example.inactive().onTrue(outtake);

        return routine;
    }

    private DoubleSupplier distToEnd(AutoTrajectory trajectory) {
        return () -> trajectory.getFinalPose().get().getTranslation().getDistance(drive.getPose().getTranslation());
    }
}
