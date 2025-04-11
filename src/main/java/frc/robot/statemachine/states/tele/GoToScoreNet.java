package frc.robot.statemachine.states.tele;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.AutoTargetUtils;
import frc.robot.util.bboard.ButtonBoard;

public class GoToScoreNet extends State {
    public GoToScoreNet(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
            Shoulder shoulder) {
        super(stateMachine);

        startWhenActive(Commands.runOnce(() -> AutoTargetUtils.setNetTarget(drive))
                .andThen(DriveCommands.goToPoint(drive, AutoTargetUtils::net)));
        startWhenActive(elevator.idle());
        startWhenActive(shoulder.idle());
    }
}
