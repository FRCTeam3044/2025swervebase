package frc.robot.statemachine.states.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoRoutines {
    public AutoRoutine testAuto() {
        AutoRoutine routine = RobotContainer.getInstance().autoFactory.newRoutine("Test");

        // Load the routine's trajectories
        AutoTrajectory example = routine.trajectory("Car");
        RobotContainer.getInstance().startPose = example.getInitialPose().get();
        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
                Commands.sequence(
                        example.resetOdometry(),
                        example.cmd()));

        example.atTime("Score").onTrue(RobotContainer.getInstance().getEndEffector().runIntake());

        return routine;
    }

}
