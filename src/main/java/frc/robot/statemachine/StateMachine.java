package frc.robot.statemachine;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.statemachine.states.DisabledState;
import frc.robot.statemachine.states.TeleState;
import frc.robot.statemachine.states.TestState;
import frc.robot.statemachine.states.tele.GoToIntake;
import frc.robot.statemachine.states.tele.GoToReefIntake;
import frc.robot.statemachine.states.tele.GoToScoreAlgae;
import frc.robot.statemachine.states.tele.GoToScoreNet;
import frc.robot.statemachine.states.tele.GoToScoreProcessor;
import frc.robot.statemachine.states.tele.GoToScoringPosition;
import frc.robot.statemachine.states.tele.GoToStationIntake;
import frc.robot.statemachine.states.tele.IntakeAlgae;
import frc.robot.statemachine.states.tele.IntakeGamePiece;
import frc.robot.statemachine.states.tele.ManualState;
import frc.robot.statemachine.states.tele.ScoreAlgae;
import frc.robot.statemachine.states.tele.ScoreAlgaeNet;
import frc.robot.statemachine.states.tele.ScoreAlgaeProcessor;
import frc.robot.statemachine.states.tele.ScoreGamePiece;
import frc.robot.statemachine.states.tele.scoreCoral.GoToScoreCoral;
import frc.robot.statemachine.states.tele.scoreCoral.IntakeCoral;
import frc.robot.statemachine.states.tele.scoreCoral.ScoreCoral;
import frc.robot.statemachine.states.tele.scoreCoral.ScoreL1;
import frc.robot.statemachine.states.tele.scoreCoral.ScoreL2;
import frc.robot.statemachine.states.tele.scoreCoral.ScoreL3;
import frc.robot.statemachine.states.tele.scoreCoral.ScoreL4;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.bboard.ButtonBoard;
import me.nabdev.oxconfig.ConfigurableParameter;

public class StateMachine extends StateMachineBase {
        public ConfigurableParameter<Double> stagingThreshold = new ConfigurableParameter<>(0.0,
                        "Staging Distance Threshold");
        public ConfigurableParameter<Double> alignmentThreshold = new ConfigurableParameter<Double>(0.0,
                        "Full Alignment Distance Threshold");
        public ConfigurableParameter<Double> algaeLeaveThreshold = new ConfigurableParameter<Double>(1.0,
                        "Algae Leave Threshold");

        public StateMachine(CommandXboxController driverController, CommandXboxController operatorController,
                        ButtonBoard buttonBoard, LoggedDashboardChooser<Command> chooser,
                        Drive drive, Elevator elevator, Shoulder shoulder, EndEffector endEffector, LEDs LEDs,
                        Climber climber) {
                super();
                State disabled = new DisabledState(this);
                currentState = disabled;

                State teleop = new TeleState(this, buttonBoard, endEffector);
                State test = new TestState(this, driverController, operatorController, elevator, shoulder, endEffector,
                                climber, LEDs,
                                drive);

                this.registerToRootState(test, teleop, disabled);

                // Teleop
                ManualState manual = new ManualState(this, driverController, operatorController, drive, elevator,
                                shoulder, endEffector, LEDs, buttonBoard, climber);
                ScoreGamePiece scoreGamePiece = new ScoreGamePiece(this);
                ScoreCoral scoreCoral = new ScoreCoral(this, LEDs);

                ScoreL1 scoreL1 = new ScoreL1(this, buttonBoard, drive, endEffector, elevator, shoulder, LEDs);
                ScoreL2 scoreL2 = new ScoreL2(this, buttonBoard, drive, endEffector, elevator, shoulder, LEDs);
                ScoreL3 scoreL3 = new ScoreL3(this, buttonBoard, drive, endEffector, elevator, shoulder, LEDs);
                ScoreL4 scoreL4 = new ScoreL4(this, buttonBoard, drive, endEffector, elevator, shoulder, LEDs);

                ScoreAlgae scoreAlgae = new ScoreAlgae(this);
                ScoreAlgaeNet scoreAlgaeNet = new ScoreAlgaeNet(this, buttonBoard, drive, endEffector, LEDs);
                ScoreAlgaeProcessor scoreAlgaeProcessor = new ScoreAlgaeProcessor(this, buttonBoard, drive, endEffector,
                                LEDs, elevator, shoulder);
                IntakeGamePiece intakeGamePiece = new IntakeGamePiece(this);
                IntakeCoral intakeCoral = new IntakeCoral(this, buttonBoard, drive, elevator, shoulder, endEffector,
                                LEDs);
                IntakeAlgae intakeAlgae = new IntakeAlgae(this, buttonBoard, drive, elevator, shoulder, endEffector,
                                LEDs);
                GoToIntake goToIntake = new GoToIntake(this);
                GoToReefIntake goToReefIntake = new GoToReefIntake(this, buttonBoard, drive, LEDs);
                GoToStationIntake goToStationIntake = new GoToStationIntake(this, buttonBoard, drive, elevator,
                                shoulder, LEDs);
                GoToScoringPosition goToScoringPosition = new GoToScoringPosition(this);
                GoToScoreCoral goToScoreCoral = new GoToScoreCoral(this, buttonBoard, drive, elevator, shoulder);
                GoToScoreAlgae goToScoreAlgae = new GoToScoreAlgae(this, LEDs);
                GoToScoreNet goToScoreNet = new GoToScoreNet(this, buttonBoard, drive);
                GoToScoreProcessor goToScoreProcessor = new GoToScoreProcessor(this, buttonBoard, drive, elevator,
                                shoulder);

                State dummyScoring = new State(this) {

                };

                State dummyCoralScoring = new State(this) {

                };

                State dummyGoToScore = new State(this) {

                };

                teleop.withModeTransitions(disabled, teleop, test)
                                .withDefaultChild(manual)
                                .withChild(goToScoringPosition)
                                .withChild(scoreGamePiece)
                                .withChild(goToIntake)
                                .withChild(intakeGamePiece);

                goToScoringPosition.withChild(goToScoreCoral, endEffector::hasCoral, 0, "Has coral")
                                .withChild(goToScoreAlgae, endEffector::hasAlgae, 1, "Has algae")
                                .withDefaultChild(dummyGoToScore);
                scoreGamePiece.withChild(scoreCoral, endEffector::hasCoral, 0, "Has coral")
                                .withChild(scoreAlgae, endEffector::hasAlgae, 1, "Has algae")
                                .withDefaultChild(dummyScoring);

                goToIntake.withChild(goToStationIntake, () -> !buttonBoard.getAlgaeMode(), 0, "Coral Mode")
                                .withChild(goToReefIntake, buttonBoard::getAlgaeMode, 1, "Algae Mode");

                intakeGamePiece.withChild(intakeCoral, () -> !buttonBoard.getAlgaeMode(), 0, "Coral Mode")
                                .withChild(intakeAlgae, buttonBoard::getAlgaeMode, 1, "Algae Mode");

                // Specific Algae intake and score
                goToScoreAlgae.withChild(goToScoreNet, () -> !buttonBoard
                                .getSelectedAlgaeLocation(), 0, "Net selected")
                                .withChild(goToScoreProcessor, buttonBoard::getSelectedAlgaeLocation, 1,
                                                "Processor selected");

                scoreAlgae.withChild(scoreAlgaeNet, () -> !buttonBoard
                                .getSelectedAlgaeLocation(), 0, "Net selected")
                                .withChild(scoreAlgaeProcessor,
                                                buttonBoard::getSelectedAlgaeLocation, 1, "Processor selected");

                scoreCoral.withDefaultChild(dummyCoralScoring)
                                .withChild(scoreL1, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L1, 0,
                                                "L1 Selected")
                                .withChild(scoreL2, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L2, 0,
                                                "L2 Selected")
                                .withChild(scoreL3, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L3, 0,
                                                "L3 Selected")
                                .withChild(scoreL4, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L4, 0,
                                                "L4 Selected");

                // Example, will be button board later
                manual.withTransition(goToScoringPosition,
                                () -> buttonBoard.fullAuto() &&
                                                driverController.rightTrigger().getAsBoolean()
                                                && (endEffector.hasCoral() || endEffector.hasAlgae())
                                                && buttonBoard.scoringSelected(),
                                "Driver presses score")
                                .withTransition(goToIntake,
                                                () -> buttonBoard.fullAuto() &&
                                                                driverController.leftTrigger().getAsBoolean()
                                                                && (!endEffector.hasCoral() && !endEffector.hasAlgae())
                                                                && buttonBoard.intakeSelected(),
                                                "Driver presses intake");

                goToScoringPosition
                                .withTransition(scoreGamePiece, () -> buttonBoard.closeToScoringTarget(drive),
                                                "Close to scoring location")
                                .withTransition(manual, () -> !driverController.rightTrigger()
                                                .getAsBoolean(), "Score button released");

                scoreGamePiece.withTransition(goToScoringPosition, () -> !buttonBoard.closeToScoringTarget(drive),
                                "Far from scoring location")
                                .withTransition(manual, () -> !driverController.rightTrigger()
                                                .getAsBoolean(), "Score button released")
                                .withTransition(manual, () -> {
                                        if (buttonBoard.getCoralReefLevel() == CoralLevel.L4
                                                        && endEffector.hasCoral()) {
                                                return endEffector.noGamePiece() && shoulder.inSafeZone();
                                        } else {
                                                return endEffector.noGamePiece();
                                        }
                                }, // && shoulder.inSafeZone(),
                                                "No game piece in robot");

                goToIntake.withTransition(intakeGamePiece, () -> buttonBoard.closeToIntakeTarget(drive),
                                "Close to intake location")
                                .withTransition(manual, () -> !driverController.leftTrigger()
                                                .getAsBoolean(), "Intake button released");

                intakeGamePiece.withTransition(goToIntake, () -> !buttonBoard.closeToIntakeTarget(drive),
                                "Intake location changed")
                                .withTransition(manual, () -> !driverController.leftTrigger()
                                                .getAsBoolean(), "Intake button released")
                                .withTransition(manual, () -> {
                                        if (buttonBoard.getAlgaeMode()) {
                                                return shoulder.getShoulderAngle() < algaeLeaveThreshold.get()
                                                                && endEffector.hasAlgae();
                                        } else {
                                                return endEffector.hasCoral();
                                        }
                                },
                                                "Game piece in robot");

                intakeCoral.withTransition(manual, () -> endEffector.hasCoral(), "Has coral");

                intakeAlgae.withTransition(manual, () -> endEffector.hasAlgae(), "Has algae");

                dummyScoring.withTransition(manual, () -> true, "Go back to manual");
                dummyCoralScoring.withTransition(manual, () -> true, "Go back to manual");
                dummyGoToScore.withTransition(manual, () -> true, "Go back to manual");

                scoreL1.withTransition(scoreL2, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L2, "L2 Selected")
                                .withTransition(scoreL3, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L3,
                                                "L3 Selected")
                                .withTransition(scoreL4, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L4,
                                                "L4 Selected");
                scoreL2.withTransition(scoreL1, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L1, "L1 Selected")
                                .withTransition(scoreL3, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L3,
                                                "L3 Selected")
                                .withTransition(scoreL4, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L4,
                                                "L4 Selected");
                scoreL3.withTransition(scoreL1, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L1, "L1 Selected")
                                .withTransition(scoreL2, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L2,
                                                "L2 Selected")
                                .withTransition(scoreL4, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L4,
                                                "L4 Selected");
                scoreL4.withTransition(scoreL1, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L1, "L1 Selected")
                                .withTransition(scoreL2, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L2,
                                                "L2 Selected")
                                .withTransition(scoreL3, () -> buttonBoard.getCoralReefLevel() == CoralLevel.L3,
                                                "L3 Selected");

                // Test
                test.withModeTransitions(disabled, teleop, test);

                // Disabled
                disabled.withModeTransitions(disabled, teleop, test);
        }
}
