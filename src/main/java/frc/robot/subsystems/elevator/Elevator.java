package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.ConfigurableLinearInterpolation;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private final ConfigurableLinearInterpolation L1 = new ConfigurableLinearInterpolation("Elevator L1 Heights");
    private final ConfigurableLinearInterpolation L2 = new ConfigurableLinearInterpolation("Elevator L2 Heights");
    private final ConfigurableLinearInterpolation L3 = new ConfigurableLinearInterpolation("Elevator L3 Heights");
    private final ConfigurableLinearInterpolation L4 = new ConfigurableLinearInterpolation("Elevator L4 Heights");

    public Elevator(ElevatorIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Command elevatorMove(DoubleSupplier speed) {
        return Commands.runEnd(() -> io.setSpeed(speed.getAsDouble()), () -> io.setSpeed(0.0), this)
                .withName("Elevator manual move");
    }

    public Command toCoral(CoralLevel level, DoubleSupplier robotDistance) {
        return Commands.run(() -> io.setPosition(getHeightForCoral(level, robotDistance.getAsDouble())), this)
                .withName("Elevator to CoralLevel");
    }

    private double getHeightForCoral(CoralLevel level, double distance) {
        switch (level) {
            case L1:
                return L1.calculate(distance);
            case L2:
                return L2.calculate(distance);
            case L3:
                return L3.calculate(distance);
            case L4:
                return L4.calculate(distance);
            default:
                return 0;
        }
    }

    public ElevatorIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public double getElevatorHeight() {
        return inputs.elevatorHeightMeters;
    }
}
