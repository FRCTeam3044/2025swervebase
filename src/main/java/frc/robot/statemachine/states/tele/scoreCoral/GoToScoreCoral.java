package frc.robot.statemachine.states.tele.scoreCoral;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachine.StateMachine;
import frc.robot.statemachine.reusable.State;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.bboard.ButtonBoard;
import me.nabdev.oxconfig.ConfigurableParameter;

public class GoToScoreCoral extends State {
        private ConfigurableParameter<Double> l4Dist = new ConfigurableParameter<Double>(1.0, "L4 Start Shoulder dist");
        private ConfigurableParameter<Double> l4DistAuto = new ConfigurableParameter<Double>(2.6,
                        "L4 Start Shoulder dist (auto)");
        private ConfigurableParameter<Double> l4FullDistAuto = new ConfigurableParameter<Double>(2.5,
                        "L4 Full Shoulder dist (auto)");

        public GoToScoreCoral(StateMachine stateMachine, ButtonBoard buttonBoard, Drive drive, Elevator elevator,
                        Shoulder shoulder) {
                super(stateMachine);

                Supplier<Rotation2d> angle = () -> {
                        Translation2d reef = buttonBoard.getCoralReefReference().getTranslation();
                        // Translation2d robot = drive.getPose().getTranslation();
                        Translation2d robot = buttonBoard.getCoralReefTarget().getTranslation();
                        Translation2d diff = reef.minus(robot);
                        return Rotation2d.fromRadians(Math.atan2(diff.getY(), diff.getX()));
                        // + (buttonBoard.getCoralReefLevel() == CoralLevel.L2 ? Math.PI : 0));
                };
                // Command ends when the target changes
                Command goToReef = DriveCommands.goToPoint(drive, buttonBoard::getCoralReefTarget, angle)
                                .until(buttonBoard::coralReefJustChanged);

                // Start command initially
                startWhenActive(goToReef);

                // When the target changed last frame, then we start the new command (else it
                // would just cancel itself).
                t(buttonBoard::coralReefJustChanged).onFalse(goToReef);

                DoubleSupplier distToTarget = buttonBoard.getCoralReefTargetDist(drive);
                DoubleSupplier distToRef = buttonBoard.getCoralReefReferenceDist(drive);
                // SmartTrigger staging = t(() -> dist.getAsDouble() <
                // stateMachine.stagingThreshold.get());
                // staging.whileTrue(Commands
                // .defer(() -> elevator.stageCoral(buttonBoard.getCoralReefLevel()),
                // Set.of(elevator))
                // .withName("Elevator to CoralLevel"));
                // staging.whileTrue(Commands
                // .defer(() -> shoulder.stageCoral(buttonBoard.getCoralReefLevel()),
                // Set.of(shoulder))
                // .withName("Shoulder to CoralLevel"));
                // staging.whileFalse(shoulder.idle());
                Command alignElevatorAndShoulder = elevator
                                .toCoralNoStop(() -> CoralLevel.L4, distToRef)
                                .alongWith(shoulder.scoreCoralNoStop(() -> CoralLevel.L4, distToRef, () -> false));
                t(() -> buttonBoard.getCoralReefLevel() == CoralLevel.L4 && distToTarget
                                .getAsDouble() < (/* DriverStation.isAutonomous() ? l4FullDistAuto.get(): */l4Dist
                                                .get()))
                                .runWhileTrue(alignElevatorAndShoulder);

                t(() -> buttonBoard.getCoralReefLevel() == CoralLevel.L4 && distToTarget.getAsDouble() < l4Dist.get())
                                .or(DriverStation::isAutonomous)
                                .runWhileFalse(shoulder.idle().alongWith(elevator.idle()));

                t(() -> buttonBoard.getCoralReefLevel() == CoralLevel.L4 && DriverStation.isAutonomous()
                                && distToTarget.getAsDouble() > l4Dist.get()
                                && distToTarget.getAsDouble() < l4DistAuto.get())
                                .whileTrue(shoulder.intakeCoral().alongWith(elevator.stageL4()));

                t(() -> buttonBoard.getCoralReefLevel() == CoralLevel.L4 && DriverStation.isAutonomous()
                                && distToTarget.getAsDouble() > l4Dist.get()
                                && distToTarget.getAsDouble() > l4DistAuto.get())
                                .whileTrue(shoulder.idle().alongWith(elevator.idle()));
                startWhenActive(shoulder.idle());
                startWhenActive(elevator.idle());
        }
}
