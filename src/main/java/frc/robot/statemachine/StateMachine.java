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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.ButtonBoardUtil;

public class StateMachine extends StateMachineBase {
        public StateMachine(CommandXboxController driverController, CommandXboxController operatorController, ButtonBoardUtil buttonBoard, LoggedDashboardChooser<Command> chooser,
                        Drive drive, Elevator elevator, Shoulder shoulder, EndEffector endEffector) {
                super();
                State disabled = new DisabledState(this);
                currentState = disabled;

                State teleop = new TeleState(this);
                State auto = new AutoState(this, chooser);
                State test = new TestState(this, driverController, elevator, shoulder, endEffector);

                this.registerToRootState(test, auto, teleop, disabled);

                // Teleop
                ManualState manual = new ManualState(this, driverController, operatorController, drive);
                ScoreGamePiece scoreGamePiece = new ScoreGamePiece(this);
                ScoreCoral scoreCoral = new ScoreCoral(this, buttonBoard, drive, endEffector);
                ScoreAlgae scoreAlgae = new ScoreAlgae(this);
                ScoreAlgaeNet scoreAlgaeNet = new ScoreAlgaeNet(this, buttonBoard, drive, endEffector);
                ScoreAlgaeProcessor scoreAlgaeProcessor = new ScoreAlgaeProcessor(this, buttonBoard, drive, endEffector);
                IntakeGamePiece intakeGamePiece = new IntakeGamePiece(this);
                IntakeCoral intakeCoral = new IntakeCoral(this, buttonBoard, drive, elevator, endEffector);
                IntakeAlgae intakeAlgae = new IntakeAlgae(this,buttonBoard, drive, elevator, endEffector);
                GoToIntake goToIntake = new GoToIntake(this);
                GoToReefIntake goToReefIntake = new GoToReefIntake(this, buttonBoard, drive);
                GoToStationIntake goToStationIntake = new GoToStationIntake(this, buttonBoard, drive);
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


                goToScoringPosition.withChild(goToScoreCoral, () -> false, 0, "Has coral")
                                .withChild(goToScoreAlgae, () -> false, 1, "Has algae");

                scoreGamePiece.withChild(scoreCoral, () -> false, 0, "Has coral")
                                .withChild(scoreAlgae, () -> false, 1, "Has algae");

                goToIntake.withChild(goToReefIntake, () -> false, 0, "Reef selected")
                                .withChild(goToStationIntake, () -> false, 1, "Station selected");

                intakeGamePiece.withChild(intakeCoral, () -> false, 0, "Reef selected")
                                .withChild(intakeAlgae, () -> false, 1, "Station selected");
                
                // Specific Algae intake and score
                goToScoreAlgae.withChild(goToScoreNet, () -> false, 0, "Net selected")
                                .withChild(goToScoreProcessor, () -> false, 1, "Processor selected");

                scoreAlgae.withChild(scoreAlgaeNet, () -> false, 0, "Net selected")
                                .withChild(scoreAlgaeProcessor, () -> false, 1, "Processor selected");

                // Example, will be button board later
                manual.withTransition(goToScoringPosition, () -> false, "Driver presses score")
                                .withTransition(goToIntake, () -> false, "Driver presses intake");

                goToScoringPosition.withTransition(scoreGamePiece, () -> false, "Close to scoring location")
                                .withTransition(manual, () -> false, "Score button released");

                scoreGamePiece.withTransition(goToScoringPosition, () -> false, "Scoring location changed")
                                .withTransition(manual, () -> false, "Score button released")
                                .withTransition(manual, () -> false, "No game piece in robot");

                goToIntake.withTransition(intakeGamePiece, () -> false, "Close to intake location")
                                .withTransition(manual, () -> false, "Intake button released");

                intakeGamePiece.withTransition(goToIntake, () -> false, "Intake location changed")
                                .withTransition(manual, () -> false, "Intake button released");

                intakeCoral.withTransition(manual, () -> false, "Has coral");

                intakeAlgae.withTransition(manual, () -> false, "Has algae");

                // Auto
                auto.withModeTransitions(disabled, teleop, auto, test);

                // Test
                test.withModeTransitions(disabled, teleop, auto, test);

                // Disabled
                disabled.withModeTransitions(disabled, teleop, auto, test);
        }
}
