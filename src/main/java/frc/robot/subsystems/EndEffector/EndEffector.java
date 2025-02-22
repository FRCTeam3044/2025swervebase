package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;

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

    public Command runIntake() {
        return runIntakeSpeed(intakeSpeed::get).withName("Run Intake");
    }

    public Command runIntakeReverse() {
        return runIntakeSpeed(() -> -intakeSpeed.get()).withName("Run Intake Reverse");
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }
}
