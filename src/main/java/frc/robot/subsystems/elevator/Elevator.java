package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.ConfigurableLinearInterpolation;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import me.nabdev.oxconfig.ConfigurableParameter;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private final ConfigurableLinearInterpolation L1 = new ConfigurableLinearInterpolation("Elevator L1 Heights");
    private final ConfigurableLinearInterpolation L2 = new ConfigurableLinearInterpolation("Elevator L2 Heights");
    private final ConfigurableLinearInterpolation L3 = new ConfigurableLinearInterpolation("Elevator L3 Heights");
    private final ConfigurableLinearInterpolation L4 = new ConfigurableLinearInterpolation("Elevator L4 Heights");

    private final ConfigurableLinearInterpolation intakeCoral = new ConfigurableLinearInterpolation(
            "Elevator Intake Heights");

    private final ConfigurableParameter<Double> elevatorTargetThreshold = new ConfigurableParameter<>(0.075,
            "Elevator Target Threshold");

    private final ConfigurableParameter<Double> idleHeight = new ConfigurableParameter<>(0.5,
            "Elevator Idle Height");

    private final BooleanSupplier shoulderInDangerZone;

    public Elevator(ElevatorIO io, BooleanSupplier shoulderInDangerZone) {
        this.io = io;
        this.shoulderInDangerZone = shoulderInDangerZone;

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
        io.updateInputs(inputs, shoulderInDangerZone.getAsBoolean());
        Logger.processInputs("Elevator", inputs);
    }

    public Command move(DoubleSupplier speed) {
        return Commands.runEnd(() -> io.setSpeed(speed.getAsDouble()), () -> io.setSpeed(0.0), this)
                .withName("Elevator manual move");
    }

    public Command toPosition(DoubleSupplier height) {
        return Commands.run(() -> io.setPosition(height.getAsDouble()), this).withName("Elevator to position");
    }

    public Command toCoral(Supplier<CoralLevel> level) {
        return Commands
                .run(() -> io.setPosition(getHeightForCoral(level.get(), getCloseDistanceForCoral(level.get()), false)),
                        this)
                .withName("Elevator to CoralLevel");
    }

    public Command toCoral(Supplier<CoralLevel> level, DoubleSupplier robotDistance) {
        return Commands
                .run(() -> io.setPosition(getHeightForCoral(level.get(), robotDistance.getAsDouble(), false)), this)
                .withName("Elevator to CoralLevel");
    }

    public Command intakeCoral(DoubleSupplier robotDistance) {
        return Commands.run(() -> io.setPosition(intakeCoral.calculate(robotDistance.getAsDouble())), this)
                .withName("Elevator to intake");
    }

    public Command intakeCoral() {
        return Commands.run(() -> io.setPosition(intakeCoral.calculate(intakeCoral.getX1())), this)
                .withName("Elevator to intake");
    }

    public Command stageIntake() {
        return Commands.run(() -> io.setPosition(intakeCoral.getY2()), this).withName("Elevator to intake");
    }

    public Command idle() {
        return Commands.run(() -> io.setPosition(idleHeight.get()), this).withName("Elevator to idle");
    }

    private double getHeightForCoral(CoralLevel level, double distance, boolean staging) {
        switch (level) {
            case L1:
                return staging ? L1.getY2() : L1.calculate(distance);
            case L2:
                return staging ? L2.getY2() : L2.calculate(distance);
            case L3:
                return staging ? L3.getY2() : L3.calculate(distance);
            case L4:
                return staging ? L4.getY2() : L4.calculate(distance);
            default:
                return 0;
        }
    }

    private double getCloseDistanceForCoral(CoralLevel level) {
        switch (level) {
            case L1:
                return L1.getX1();
            case L2:
                return L2.getX1();
            case L3:
                return L3.getX1();
            case L4:
                return L4.getX1();
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

    public boolean isAtTarget() {
        return Math.abs(inputs.elevatorHeightMeters - inputs.setpointMeters) < elevatorTargetThreshold.get();
    }
}
