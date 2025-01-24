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
        return Commands.runEnd(() -> {io.setAlgaeSpeed(desiredSpeed.getAsDouble()); io.setCoralSpeed(desiredSpeed.getAsDouble());}, () -> {io.setAlgaeSpeed(0.0); io.setCoralSpeed(0.0);});
    }

    public Command runIntake(){
        return runIntakeSpeed(intakeSpeed::get);
    }

    public Command runIntakeReverse(){
        return runIntakeSpeed(() -> -intakeSpeed.get());
    }

    public Command runUntilSpike(DoubleSupplier desiredSpeed) {
        return Commands.none();
    }
}
