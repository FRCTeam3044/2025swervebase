package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.statemachine.reusable.State;
import frc.robot.statemachine.reusable.StateMachineBase;
import frc.robot.statemachine.states.AutoState;
import frc.robot.statemachine.states.DisabledState;
import frc.robot.statemachine.states.TeleState;
import frc.robot.statemachine.states.TestState;
import frc.robot.statemachine.states.tele.IntakeAlgae;
import frc.robot.statemachine.states.tele.IntakeCoral;
import frc.robot.statemachine.states.tele.IntakeGamePiece;
import frc.robot.statemachine.states.tele.ManualState;
import frc.robot.statemachine.states.tele.ScoreAlgae;
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
                IntakeGamePiece intakeGamePiece = new IntakeGamePiece(this);
                IntakeCoral intakeCoral = new IntakeCoral(this);
                IntakeAlgae intakeAlgae = new IntakeAlgae(this);
                ScoreGamePiece scoreGamePiece = new ScoreGamePiece(this);
                ScoreCoral scoreCoral = new ScoreCoral(this);
                ScoreAlgae scoreAlgae = new ScoreAlgae(this);

                teleop.withModeTransitions(disabled, teleop, auto, test)
                                .withDefaultChild(manual);

                // Example, will be button board later
                manual.withTransition(scoreGamePiece, () -> false, "Example");

                // Auto
                auto.withModeTransitions(disabled, teleop, auto, test);

                // Test
                test.withModeTransitions(disabled, teleop, auto, test);

                // Disabled
                disabled.withModeTransitions(disabled, teleop, auto, test);
        }
}
