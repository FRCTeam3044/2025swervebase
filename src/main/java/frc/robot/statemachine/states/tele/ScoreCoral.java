package frc.robot.statemachine.states.tele;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.ButtonBoardUtil;

public class ScoreCoral extends State {
    public ScoreCoral(StateMachineBase stateMachine, ButtonBoardUtil buttonBoard, Drive drive, EndEffector endEffector,
            Elevator elevator, Shoulder shoulder, LEDs LEDs) {
        super(stateMachine);

        startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getCoralReefTarget));
        startWhenActive(Commands.deferredProxy(
                () -> elevator.toCoral(buttonBoard.getCoralReefLevel(), buttonBoard.getCoralReefTargetDist(drive))));
        startWhenActive(Commands.deferredProxy(
                () -> shoulder.scoreCoral(buttonBoard.getCoralReefLevel(), buttonBoard.getCoralReefTargetDist(drive))));
        startWhenActive(endEffector.runIntakeReverse());
        startWhenActive(LEDs.intakingAndScoringCoral());
    }
}
