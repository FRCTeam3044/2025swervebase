package frc.robot.statemachine.states.tele;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;

public class ScoreCoral extends State {
        public ScoreCoral(StateMachineBase stateMachine, ButtonBoard buttonBoard, Drive drive, EndEffector endEffector,
                        Elevator elevator, Shoulder shoulder, LEDs LEDs) {
                super(stateMachine);

                DoubleSupplier distanceToRef = buttonBoard.getCoralReefReferenceDist(drive);
                startWhenActive(DriveCommands.pointControl(drive, buttonBoard::getCoralReefTarget));

                BooleanSupplier staging = () -> !elevator.isAtTarget();
                startWhenActive(Commands
                                .defer(() -> elevator.toCoral(buttonBoard.getCoralReefLevel(), distanceToRef),
                                                Set.of(elevator))
                                .withName("Elevator to CoralLevel"));
                startWhenActive(Commands
                                .defer(() -> shoulder.scoreCoral(buttonBoard.getCoralReefLevel(), distanceToRef,
                                                staging), Set.of(shoulder))
                                .withName("Shoulder to CoralLevel"));

                DoubleSupplier distanceToTarget = buttonBoard.getCoralReefTargetDist(drive);
                BooleanSupplier readyToScore = () -> distanceToTarget.getAsDouble() < StateMachine.alignmentThreshold
                                .get() && elevator.isAtTarget()
                                && shoulder.isAtCoralTarget(buttonBoard::getCoralReefLevel, distanceToRef);
                t(readyToScore).whileTrue(endEffector.runIntakeReverse());
                startWhenActive(LEDs.intakingAndScoringCoral());
        }
}
