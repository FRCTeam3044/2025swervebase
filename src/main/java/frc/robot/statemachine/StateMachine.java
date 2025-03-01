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
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.bboard.ButtonBoard;
import me.nabdev.oxconfig.ConfigurableParameter;

public class StateMachine extends StateMachineBase {
        public ConfigurableParameter<Double> stagingThreshold = new ConfigurableParameter<>(0.0,
                        "Staging Distance Threshold");
        public ConfigurableParameter<Double> alignmentThreshold = new ConfigurableParameter<Double>(0.0,
                        "Full Alignment Distance Threshold");

        public StateMachine(CommandXboxController driverController, CommandXboxController operatorController,
                        ButtonBoard buttonBoard, LoggedDashboardChooser<Command> chooser,
                        Drive drive, Elevator elevator, Shoulder shoulder, EndEffector endEffector, LEDs LEDs,
                        Climber climber) {
                super();
                State disabled = new DisabledState(this);
                currentState = disabled;

                State teleop = new TeleState(this, buttonBoard, endEffector);
                State auto = new AutoState(this, chooser);
                State test = new TestState(this, driverController, elevator, shoulder, endEffector, LEDs);

                this.registerToRootState(test, auto, teleop, disabled);

                // Teleop
                ManualState manual = new ManualState(this, driverController, operatorController, drive, elevator,
                                shoulder, endEffector, LEDs, buttonBoard, climber);
                ScoreGamePiece scoreGamePiece = new ScoreGamePiece(this);
                ScoreCoral scoreCoral = new ScoreCoral(this, buttonBoard, drive, endEffector, elevator, shoulder, LEDs);
                ScoreAlgae scoreAlgae = new ScoreAlgae(this);
                ScoreAlgaeNet scoreAlgaeNet = new ScoreAlgaeNet(this, buttonBoard, drive, endEffector, LEDs);
                ScoreAlgaeProcessor scoreAlgaeProcessor = new ScoreAlgaeProcessor(this, buttonBoard, drive, endEffector,
                                LEDs);
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
                                .withTransition(manual, () -> !endEffector.hasCoral() && !endEffector.hasAlgae(),
                                                "No game piece in robot");

                goToIntake.withTransition(intakeGamePiece, () -> buttonBoard.closeToIntakeTarget(drive),
                                "Close to intake location")
                                .withTransition(manual, () -> !driverController.leftTrigger()
                                                .getAsBoolean(), "Intake button released");

                intakeGamePiece.withTransition(goToIntake, () -> !buttonBoard.closeToIntakeTarget(drive),
                                "Intake location changed")
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
