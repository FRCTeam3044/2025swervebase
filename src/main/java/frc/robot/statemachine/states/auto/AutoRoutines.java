package frc.robot.statemachine.states.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffector.EndEffector;
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
        AutoTrajectory example = routine.trajectory("Car", 0);
        AutoTrajectory examplePart2 = routine.trajectory("Car", 1);
        AutoTrajectory examplePart3 = routine.trajectory("Car", 2);
        RobotContainer.getInstance().startPose = example.getInitialPose().get();
        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
                Commands.sequence(
                        example.resetOdometry(),
                        Commands.runOnce(drive::stop),
                        Commands.deadline(
                                Commands.waitUntil(elevator::isAtTarget)
                                        .andThen(endEffector.runIntakeReverse().until(() -> !endEffector.hasCoral())),
                                elevator.toCoral(CoralLevel.L2, () -> {
                                    return example.collectEventPoses("Score")[0].getTranslation()
                                            .getDistance(drive.getPose().getTranslation());
                                })),
                        example.cmd(),
                        Commands.runOnce(drive::stop),
                        endEffector.runIntake().until(() -> endEffector.hasCoral()),
                        examplePart2.cmd(),
                        Commands.runOnce(drive::stop),
                        Commands.waitSeconds(1),
                        examplePart3.cmd(),
                        Commands.runOnce(drive::stop),
                        Commands.waitSeconds(1)));

        // example.atTime("Score").onTrue(endEffector.runIntakeReverse().until(() ->
        // !endEffector.hasCoral()));

        return routine;
    }
}
