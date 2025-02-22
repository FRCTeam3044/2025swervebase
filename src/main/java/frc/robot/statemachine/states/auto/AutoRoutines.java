package frc.robot.statemachine.states.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoRoutines {
    public AutoRoutine testAuto() {
        AutoRoutine routine = RobotContainer.getInstance().autoFactory.newRoutine("Test");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine.trajectory("Marcus");
        RobotContainer.getInstance().startPose = driveToMiddle.getInitialPose().get();
        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
                Commands.sequence(
                        driveToMiddle.resetOdometry(),
                        driveToMiddle.cmd()));

        return routine;
    }

}
