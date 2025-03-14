package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import me.nabdev.oxconfig.ConfigurableParameter;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final ConfigurableParameter<Double> speed = new ConfigurableParameter<>(0.5, "Climber Speed");

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Command move(DoubleSupplier speed) {
        return Commands.runEnd(() -> io.setSpeed(speed.getAsDouble()), () -> io.setSpeed(0.0), this);
    }

    public Command up() {
        return servoOpen().alongWith(move(() -> speed.get())).finallyDo(() -> io.setServoClosed(true));
    }

    public Command down() {
        return Commands.parallel(servoClose(), move(() -> -speed.get()));
    }

    public Command servoOpen() {
        return Commands.run(() -> io.setServoClosed(false));
    }

    public Command servoClose() {
        return Commands.run(() -> io.setServoClosed(true));
    }

    public Command servoForward() {
        return Commands.runEnd(() -> io.setSpeed(0.5), () -> io.setSpeed(0));
    }

    public Command servoBackward() {
        return Commands.runEnd(() -> io.setSpeed(-0.5), () -> io.setSpeed(0));
    }
}
