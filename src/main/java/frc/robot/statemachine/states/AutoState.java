package frc.robot.statemachine.states;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import me.nabdev.oxconfig.ConfigurableParameter;

public class AutoState extends State {
    public static ConfigurableParameter<Double> distanceThreshold = new ConfigurableParameter<Double>(0.3,
            "Auto Start threshold");

    public AutoState(StateMachineBase stateMachine, Drive drive, AutoChooser autoChooser) {
        super(stateMachine);
        startWhenActive(DriveCommands.goToPoint(drive, () -> RobotContainer.getInstance().startPose));
        BooleanSupplier closeToTarget = () -> {
            Pose2d target = RobotContainer.getInstance().startPose;
            if (target == null) {
                return false;
            }
            return drive.getPose().getTranslation().getDistance(target.getTranslation()) < distanceThreshold.get();
        };

        t(closeToTarget).onTrue(autoChooser.selectedCommandScheduler());
    }

    @Override
    public void onEnter() {
        super.onEnter();
    }
}
