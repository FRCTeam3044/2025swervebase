package frc.robot.subsystems.EndEffector;

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

    public Command runIntake(DoubleSupplier desiredSpeed) {
        return Commands.run(() -> {io.setAlgaeSpeed(desiredSpeed.getAsDouble()); io.setCoralSpeed(desiredSpeed.getAsDouble());});
    }

    public Command runUntilSpike(DoubleSupplier desiredSpeed) {
        return Commands.none();
    }
}
