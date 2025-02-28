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

    private final ConfigurableParameter<Double> servoClosed = new ConfigurableParameter<>(0.0, "Servo Closed Position");
    private final ConfigurableParameter<Double> servoOpen = new ConfigurableParameter<>(1.0, "Servo Open Position");
    private final ConfigurableParameter<Double> speed = new ConfigurableParameter<>(0.5, "Climber Speed");
    private final ConfigurableParameter<Double> servoTolerance = new ConfigurableParameter<>(5.0, "Servo Tolerance");

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    private Command move(DoubleSupplier speed) {
        return Commands.runEnd(() -> io.setSpeed(speed.getAsDouble()), () -> io.setSpeed(0.0), this);
    }

    public Command up() {
        return servoOpen().andThen(move(() -> speed.get())).finallyDo(() -> io.setServoAngle(servoClosed.get()));
    }

    public Command down() {
        return Commands.waitUntil(() -> Math.abs(inputs.servoPosition - servoClosed.get()) < servoTolerance.get())
                .andThen(move(() -> -speed.get()));
    }

    public Command servoOpen() {
        return Commands.run(() -> io.setServoAngle(servoOpen.get()), this)
                .until(() -> Math.abs(inputs.servoPosition - servoOpen.get()) < servoTolerance.get());
    }

    public Command servoClose() {
        return Commands.run(() -> io.setServoAngle(servoClosed.get()), this)
                .until(() -> Math.abs(inputs.servoPosition - servoClosed.get()) < servoTolerance.get());
    }
}
