package frc.robot.statemachine.states.tele.scoreCoral;

import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.reusable.State;
import frc.robot.subsystems.LEDs.LEDs;

public class ScoreCoral extends State {
        public ScoreCoral(StateMachine stateMachine, LEDs LEDs) {
                super(stateMachine);
                startWhenActive(LEDs.intakingAndScoringCoral());
        }
}
