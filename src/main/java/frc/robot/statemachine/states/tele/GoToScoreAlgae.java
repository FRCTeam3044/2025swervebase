package frc.robot.statemachine.states.tele;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.LEDs.LEDs;

public class GoToScoreAlgae extends State {
    public GoToScoreAlgae(StateMachineBase stateMachine, LEDs leds) {
        super(stateMachine);
        startWhenActive(leds.goingToAlgaeIntake());
    }
}
