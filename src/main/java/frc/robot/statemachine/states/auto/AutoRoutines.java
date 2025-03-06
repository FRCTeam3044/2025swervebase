package frc.robot.statemachine.states.auto;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoFactory;
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

        private AutoFactory autoFactory;

        public AutoRoutines(Drive drive, Elevator elevator, EndEffector endEffector, Shoulder shoulder) {
                this.drive = drive;
                this.elevator = elevator;
                this.endEffector = endEffector;
                this.shoulder = shoulder;
                autoFactory = new AutoFactory(
                                drive::getPose,
                                drive::resetOdometry,
                                drive::choreoDriveController,
                                true,
                                drive);

                autoFactory.bind("stageL4", shoulder.stageCoral(CoralLevel.L4));
                autoFactory.bind("stageL3", shoulder.stageCoral(CoralLevel.L3));
                autoFactory.bind("stageL2", shoulder.stageCoral(CoralLevel.L2));
                autoFactory.bind("scorel4", getScoreCommand(CoralLevel.L4));
                autoFactory.bind("scorel3", getScoreCommand(CoralLevel.L3));
                autoFactory.bind("scorel2", getScoreCommand(CoralLevel.L2));

        }

        private Command getScoreCommand(CoralLevel level) {
                Command moveElevator = elevator.toCoral(() -> level);
                Command stageShoulder = shoulder.stageCoral(level);

                Command prep = stageShoulder.until(elevator::isAtTarget);

                Command scoreShoulder = shoulder.scoreCoral(() -> level);
                Command score = Commands.parallel(scoreShoulder,
                                Commands.waitUntil(() -> shoulder.isAtCoralTarget(() -> level))
                                                .andThen(endEffector.coralOut()));

                return Commands.deadline(prep.andThen(score.until(endEffector::noGamePiece)), moveElevator,
                                Commands.runOnce(drive::stop));
        }

        public AutoRoutine testAuto() {
                AutoRoutine routine = autoFactory.newRoutine("Test");

                // Load the routine's trajectories
                AutoTrajectory goToFirstScore = routine.trajectory("Testy Testy", 0);
                AutoTrajectory goToFirstIntake = routine.trajectory("Testy Testy", 1);
                AutoTrajectory goToSecondScore = routine.trajectory("Testy Testy", 2);

                Command intake = Commands.runOnce(drive::stop)
                                .andThen(shoulder.intakeCoral())
                                .andThen(endEffector.coralIn())
                                .until(endEffector::hasCoral)
                                .withName("Intake");

                RobotContainer.getInstance().startPose = goToFirstScore.getInitialPose().get();
                // When the routine begins, reset odometry and start the first trajectory (1)
                routine.active().onTrue(goToFirstScore.resetOdometry().andThen(goToFirstScore.cmd()));

                goToFirstScore.inactive().and(endEffector::noGamePiece).onTrue(goToFirstIntake.cmd());
                goToFirstIntake.inactive().and(endEffector::hasCoral).onTrue(goToSecondScore.cmd());
                return routine;
        }
}
