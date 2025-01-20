package frc.robot.statemachine;

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
import frc.robot.subsystems.drive.Drive;

public class StateMachine extends StateMachineBase {
        public StateMachine(CommandXboxController driverController, CommandXboxController operatorController,
                        Drive drive) {
                super();
                State disabled = new DisabledState(this);
                currentState = disabled;

                State teleop = new TeleState(this);
                State auto = new AutoState(this);
                State test = new TestState(this, driverController);

                this.registerToRootState(test, auto, teleop, disabled);

                // Teleop
                ManualState manual = new ManualState(this, driverController, operatorController, drive);
                ScoreGamePiece scoreGamePiece = new ScoreGamePiece(this);
                ScoreCoral scoreCoral = new ScoreCoral(this);
                ScoreAlgae scoreAlgae = new ScoreAlgae(this);
                ScoreAlgaeNet scoreAlgaeNet = new ScoreAlgaeNet(this);
                ScoreAlgaeProcessor scoreAlgaeProcessor = new ScoreAlgaeProcessor(this);
                IntakeGamePiece intakeGamePiece = new IntakeGamePiece(this);
                IntakeCoral intakeCoral = new IntakeCoral(this);
                IntakeAlgae intakeAlgae = new IntakeAlgae(this);
                GoToIntake goToIntake = new GoToIntake(this);
                GoToReefIntake goToReefIntake = new GoToReefIntake(this);
                GoToStationIntake goToStationIntake = new GoToStationIntake(this);
                GoToScoringPosition goToScoringPosition = new GoToScoringPosition(this);
                GoToScoreCoral goToScoreCoral = new GoToScoreCoral(this);
                GoToScoreAlgae goToScoreAlgae = new GoToScoreAlgae(this);
                GoToScoreNet goToScoreNet = new GoToScoreNet(this);
                GoToScoreProcessor goToScoreProcessor = new GoToScoreProcessor(this);

                teleop.withModeTransitions(disabled, teleop, auto, test)
                                .withDefaultChild(manual)
                                .withChild(goToScoringPosition)
                                .withChild(scoreGamePiece)
                                .withChild(goToIntake)
                                .withChild(intakeGamePiece);

                // Children inside children
                goToScoringPosition.withChild(goToScoreCoral)
                                .withChild(goToScoreAlgae);

                scoreGamePiece.withChild(scoreCoral)
                                .withChild(scoreAlgae);

                goToIntake.withChild(goToReefIntake)
                                .withChild(goToStationIntake);

                intakeGamePiece.withChild(intakeCoral)
                                .withChild(intakeAlgae);
                
                // Specific Algae intake and score
                goToScoreAlgae.withChild(goToScoreNet)
                                .withChild(goToScoreProcessor);

                scoreAlgae.withChild(scoreAlgaeNet)
                                .withChild(scoreAlgaeProcessor);

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
