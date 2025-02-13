package frc.robot.statemachine;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.statemachine.states.AutoState;
import frc.robot.statemachine.states.DisabledState;
import frc.robot.statemachine.states.TeleState;
import frc.robot.statemachine.states.TestState;
import frc.robot.statemachine.states.tele.GoToIntake;
import frc.robot.statemachine.states.tele.GoToReefIntake;
import frc.robot.statemachine.states.tele.GoToScoreAlgae;
import frc.robot.statemachine.states.tele.GoToScoreCoral;
import frc.robot.statemachine.states.tele.GoToScoreNet;
import frc.robot.statemachine.states.tele.GoToScoreProcessor;
import frc.robot.statemachine.states.tele.GoToScoringPosition;
import frc.robot.statemachine.states.tele.GoToStationIntake;
import frc.robot.statemachine.states.tele.IntakeAlgae;
import frc.robot.statemachine.states.tele.IntakeCoral;
import frc.robot.statemachine.states.tele.IntakeGamePiece;
import frc.robot.statemachine.states.tele.ManualState;
import frc.robot.statemachine.states.tele.ScoreAlgae;
import frc.robot.statemachine.states.tele.ScoreAlgaeNet;
import frc.robot.statemachine.states.tele.ScoreAlgaeProcessor;
import frc.robot.statemachine.states.tele.ScoreCoral;
import frc.robot.statemachine.states.tele.ScoreGamePiece;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.ButtonBoardUtil;

public class StateMachine extends StateMachineBase {
        public StateMachine(CommandXboxController driverController, CommandXboxController operatorController,
                        ButtonBoardUtil buttonBoard, LoggedDashboardChooser<Command> chooser,
                        Drive drive, Elevator elevator, Shoulder shoulder, EndEffector endEffector, LEDs LEDs) {
                super();
                State disabled = new DisabledState(this);
                currentState = disabled;

                State teleop = new TeleState(this);
                State auto = new AutoState(this, chooser);
                State test = new TestState(this, driverController, elevator, shoulder, endEffector);

                this.registerToRootState(test, auto, teleop, disabled);

                // Teleop
                ManualState manual = new ManualState(this, driverController, operatorController, drive, elevator, LEDs);
                ScoreGamePiece scoreGamePiece = new ScoreGamePiece(this);
                ScoreCoral scoreCoral = new ScoreCoral(this, buttonBoard, drive, endEffector, elevator, shoulder, LEDs);
                ScoreAlgae scoreAlgae = new ScoreAlgae(this);
                ScoreAlgaeNet scoreAlgaeNet = new ScoreAlgaeNet(this, buttonBoard, drive, endEffector, LEDs);
                ScoreAlgaeProcessor scoreAlgaeProcessor = new ScoreAlgaeProcessor(this, buttonBoard, drive, endEffector,
                                LEDs);
                IntakeGamePiece intakeGamePiece = new IntakeGamePiece(this);
                IntakeCoral intakeCoral = new IntakeCoral(this, buttonBoard, drive, elevator, endEffector, LEDs);
                IntakeAlgae intakeAlgae = new IntakeAlgae(this, buttonBoard, drive, elevator, endEffector, LEDs);
                GoToIntake goToIntake = new GoToIntake(this);
                GoToReefIntake goToReefIntake = new GoToReefIntake(this, buttonBoard, drive, LEDs);
                GoToStationIntake goToStationIntake = new GoToStationIntake(this, buttonBoard, drive, LEDs);
                GoToScoringPosition goToScoringPosition = new GoToScoringPosition(this);
                GoToScoreCoral goToScoreCoral = new GoToScoreCoral(this, buttonBoard, drive);
                GoToScoreAlgae goToScoreAlgae = new GoToScoreAlgae(this);
                GoToScoreNet goToScoreNet = new GoToScoreNet(this, buttonBoard, drive);
                GoToScoreProcessor goToScoreProcessor = new GoToScoreProcessor(this, buttonBoard, drive);

                teleop.withModeTransitions(disabled, teleop, auto, test)
                                .withDefaultChild(manual)
                                .withChild(goToScoringPosition)
                                .withChild(scoreGamePiece)
                                .withChild(goToIntake)
                                .withChild(intakeGamePiece);

                goToScoringPosition.withChild(goToScoreCoral, endEffector::hasCoral, 0, "Has coral")
                                .withChild(goToScoreAlgae, endEffector::hasAlgae, 1, "Has algae");

                scoreGamePiece.withChild(scoreCoral, endEffector::hasCoral, 0, "Has coral")
                                .withChild(scoreAlgae, endEffector::hasAlgae, 1, "Has algae");

                goToIntake.withChild(goToReefIntake, buttonBoard::getAlgaeMode, 0, "Reef selected")
                                .withChild(goToStationIntake, () -> !buttonBoard.getAlgaeMode(), 1, "Station selected");

                intakeGamePiece.withChild(intakeCoral, buttonBoard::getAlgaeMode, 0, "Reef selected")
                                .withChild(intakeAlgae, () -> !buttonBoard.getAlgaeMode(), 1, "Station selected");

                // Specific Algae intake and score
                goToScoreAlgae.withChild(goToScoreNet, () -> !buttonBoard
                                .getSelectedAlgaeLocation(), 0, "Net selected")
                                .withChild(goToScoreProcessor, buttonBoard::getSelectedAlgaeLocation, 1,
                                                "Processor selected");

                scoreAlgae.withChild(scoreAlgaeNet, () -> !buttonBoard
                                .getSelectedAlgaeLocation(), 0, "Net selected")
                                .withChild(scoreAlgaeProcessor,
                                                buttonBoard::getSelectedAlgaeLocation, 1, "Processor selected");

                // Example, will be button board later
                manual.withTransition(goToScoringPosition,
                                () -> driverController.rightTrigger().getAsBoolean()
                                                && (endEffector.hasCoral() || endEffector.hasAlgae()),
                                "Driver presses score")
                                .withTransition(goToIntake,
                                                () -> driverController.leftTrigger().getAsBoolean()
                                                                && (!endEffector.hasCoral() && !endEffector.hasAlgae()),
                                                "Driver presses intake");

                goToScoringPosition
                                .withTransition(scoreGamePiece, () -> buttonBoard.closeToScoringTarget(drive),
                                                "Close to scoring location")
                                .withTransition(manual, () -> !driverController.rightTrigger()
                                                .getAsBoolean(), "Score button released");

                scoreGamePiece.withTransition(goToScoringPosition, () -> false, "Scoring location changed")
                                .withTransition(manual, () -> !driverController.rightTrigger()
                                                .getAsBoolean(), "Score button released")
                                .withTransition(manual, () -> !endEffector.hasCoral() && !endEffector.hasAlgae(),
                                                "No game piece in robot");

                goToIntake.withTransition(intakeGamePiece, () -> buttonBoard.closeToIntakeTarget(drive),
                                "Close to intake location")
                                .withTransition(manual, () -> !driverController.leftTrigger()
                                                .getAsBoolean(), "Intake button released");

                intakeGamePiece.withTransition(goToIntake, () -> false, "Intake location changed")
                                .withTransition(manual, () -> !driverController.leftTrigger()
                                                .getAsBoolean(), "Intake button released")
                                .withTransition(manual, () -> endEffector.hasCoral() || endEffector.hasAlgae(),
                                                "Game piece in robot");

                intakeCoral.withTransition(manual, () -> endEffector.hasCoral(), "Has coral");

                intakeAlgae.withTransition(manual, () -> endEffector.hasAlgae(), "Has algae");

                // Auto
                auto.withModeTransitions(disabled, teleop, auto, test);

                // Test
                test.withModeTransitions(disabled, teleop, auto, test);

                // Disabled
                disabled.withModeTransitions(disabled, teleop, auto, test);
        }
}
