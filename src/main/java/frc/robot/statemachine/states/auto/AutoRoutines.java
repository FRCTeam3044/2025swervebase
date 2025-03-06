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
                AutoTrajectory path = routine.trajectory("Testy Testy");
                AutoTrajectory part1 = routine.trajectory("Testy Testy", 0);
                AutoTrajectory part2 = routine.trajectory("Testy Testy", 1);
                AutoTrajectory part3 = routine.trajectory("Testy Testy", 2);

                Command score = Commands.runOnce(drive::stop)
                                .andThen(elevator.toCoral(() -> CoralLevel.L4)
                                                .alongWith(Commands.waitUntil(elevator::isAtTarget)
                                                                .andThen(shoulder.scoreCoral(() -> CoralLevel.L4)
                                                                                .alongWith(
                                                                                                Commands.waitUntil(
                                                                                                                () -> shoulder.isAtCoralTarget(
                                                                                                                                () -> CoralLevel.L4))
                                                                                                                .andThen(endEffector
                                                                                                                                .coralOut()))))
                                                .until(() -> !endEffector.hasCoral())
                                                .withName("Score"));

                Command intake = Commands.runOnce(drive::stop)
                                .andThen(shoulder.intakeCoral())
                                .andThen(endEffector.coralIn())
                                .until(endEffector::hasCoral)
                                .withName("Intake");

                Command stageShoulder = shoulder.stageCoral(CoralLevel.L4)
                                .withName("Stage shoulder");

                RobotContainer.getInstance().startPose = path.getInitialPose().get();
                // When the routine begins, reset odometry and start the first trajectory (1)
                routine.active().onTrue(
                                Commands.sequence(
                                                path.resetOdometry(),
                                                score,
                                                part1.cmd(),
                                                Commands.runOnce(drive::stop)));

                return routine;
        }

        private DoubleSupplier distToEnd(AutoTrajectory trajectory) {
                return () -> trajectory.getFinalPose().get().getTranslation()
                                .getDistance(drive.getPose().getTranslation());
        }
}
