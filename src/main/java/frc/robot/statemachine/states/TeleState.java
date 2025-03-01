package frc.robot.statemachine.states;

import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.util.bboard.ButtonBoard;

public class TeleState extends State {
    public TeleState(StateMachineBase stateMachine, ButtonBoard bboard, EndEffector endEffector) {
        super(stateMachine);

        t(bboard::extraFour).whileTrue(endEffector.algaeOut());
        t(bboard::extraFour).whileTrue(endEffector.algaeIn());
    }
}
