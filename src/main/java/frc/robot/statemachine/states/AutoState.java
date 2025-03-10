package frc.robot.statemachine.states;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;

public class AutoState extends State {
    public AutoState(StateMachineBase stateMachine, LoggedDashboardChooser<Command> chooser) {
        super(stateMachine);
        startWhenActive(chooser::get);
    }

    @Override
    public void onEnter() {
        super.onEnter();
    }
}
