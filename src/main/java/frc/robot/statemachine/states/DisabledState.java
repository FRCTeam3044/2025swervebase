package frc.robot.statemachine.states;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.vision.Vision;

public class DisabledState extends State {
    public DisabledState(StateMachineBase stateMachine, LEDs LEDs, Vision vision) {
        super(stateMachine);
        SmartDashboard.putBoolean("LEDs in disabled", true);
        startWhenActive(LEDs.Default().ignoringDisable(true));
        t(vision::hasTarget).whileTrue(LEDs.aprilTagDetected().ignoringDisable(true))
                .whileFalse(LEDs.Default().ignoringDisable(true));
        t(() -> !SmartDashboard.getBoolean("LEDs in disabled", true)).whileTrue(LEDs.off());
    }
}
