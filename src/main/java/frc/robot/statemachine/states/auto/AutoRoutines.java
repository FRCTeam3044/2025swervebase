package frc.robot.statemachine.states.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoRoutines {
    private final RobotContainer robotContainer = new RobotContainer();

    public AutoRoutine pickupAndScoreAuto() {
        AutoRoutine routine = robotContainer.autoFactory.newRoutine("taxi");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine.trajectory("Marcus");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
                Commands.sequence(
                        driveToMiddle.resetOdometry(),
                        driveToMiddle.cmd()));

        return routine;
    }
}
