package frc.robot.subsystems.endEffector;

import static frc.robot.subsystems.endEffector.EndEffectorConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

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

    public Command coralIn() {
        return runIntakeSpeed(coralInSpeed::get).withName("Run End Effector Coral In");
    }

    public Command coralOut() {
        return runIntakeSpeed(() -> -coralOutSpeed.get()).withName("Run End Effector Coral Out");
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }
}
