package frc.robot.statemachine.states.auto;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;

public class AutoRoutines {
    private Drive drive;
    private Elevator elevator;
    private EndEffector endEffector;
    private Shoulder shoulder;

    public AutoRoutines(Drive drive, Elevator elevator, EndEffector endEffector, Shoulder shoulder) {
        this.drive = drive;
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.shoulder = shoulder;
    }

    public AutoRoutine testAuto() {
        AutoRoutine routine = RobotContainer.getInstance().autoFactory.newRoutine("Test");

        // Load the routine's trajectories
        AutoTrajectory example = routine.trajectory("Testy Testy");
        AutoTrajectory intaking = routine.trajectory("Testy Testy", 0);
        AutoTrajectory scoring = routine.trajectory("Testy Testy", 1);
        Command outtake = Commands.waitUntil(elevator::isAtTarget)
                .andThen(endEffector::runIntakeReverse).until(() -> !endEffector.hasCoral())
                .withName("Outtake when elevator ready");

        Command intake = Commands.run(endEffector::runIntake).withName("Intake");

        RobotContainer.getInstance().startPose = intaking.getInitialPose().get();
        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
                Commands.sequence(
                        example.resetOdometry(),
                        example.cmd(),
                        Commands.runOnce(drive::stop)));

        scoring.atTime("elevator").onTrue(
                Commands.runOnce(drive::stop)
                        .andThen(elevator.toCoral(() -> CoralLevel.L4, distToEnd(example)))
                        .until(() -> !endEffector.hasCoral())
                        .andThen(elevator.idle()));

        scoring.atTime("stage_shoulder").onTrue(shoulder.stageCoral(CoralLevel.L4));

        example.atTime("intake").onTrue(
                Commands.runOnce(drive::stop)
                        .andThen(elevator.intakeCoral()).andThen(intake).until(endEffector::hasCoral)
                        .andThen(elevator.idle().alongWith(scoring.cmd())));

        example.inactive().onTrue(outtake);

        return routine;
    }

    private DoubleSupplier distToEnd(AutoTrajectory trajectory) {
        return () -> trajectory.getFinalPose().get().getTranslation().getDistance(drive.getPose().getTranslation());
    }
}
