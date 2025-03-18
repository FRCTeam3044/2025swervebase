package frc.robot.subsystems.endEffector;

import static frc.robot.subsystems.endEffector.EndEffectorConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private final Debouncer noGamePieceDebouncer = new Debouncer(0.5, DebounceType.kRising);

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }

    public Command runIntakeSpeed(DoubleSupplier desiredSpeed) {
        return Commands.runEnd(() -> io.setSpeed(desiredSpeed.getAsDouble()), () -> io.setSpeed(0.0), this)
                .withName("Run Intake Speed");
    }

    public Command algaeIn() {
        return runIntakeSpeed(algaeInSpeed::get).withName("Run End Effector Algae In");
    }

    public Command algaeOut() {
        return runIntakeSpeed(() -> -algaeOutSpeed.get()).withName("Run End Effector Algae Out");
    }

    public Command algaeOutNet() {
        return runIntakeSpeed(() -> -algaeOutSpeedNet.get()).withName("Run End Effector Algae Out (Net)");
    }

    public Command coralIn() {
        return runIntakeSpeed(coralInSpeed::get).withName("Run End Effector Coral In");
    }

    public Command coralOut() {
        return runIntakeSpeed(() -> -coralOutSpeed.get()).withName("Run End Effector Coral Out");
    }

    public Command coralOutSlow() {
        return runIntakeSpeed(() -> -slowCoralOutSpeed.get()).withName("Run End Effector Coral Out Slow");
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }

    @AutoLogOutput(key = "NoGamePiece")
    public boolean noGamePiece() {
        boolean hasNoGamePiece = !hasCoral() && !hasAlgae();
        return noGamePieceDebouncer.calculate(hasNoGamePiece);
    }
}
