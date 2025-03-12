package frc.robot.statemachine.states;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.vision.Vision;

public class DisabledState extends State {
    public DisabledState(StateMachineBase stateMachine, LEDs LEDs, Vision vision) {
        super(stateMachine);
        startWhenActive(LEDs.Default());
        t(vision::hasTarget).whileTrue(LEDs.aprilTagDetected()).whileFalse(LEDs.Default());
    }
}
