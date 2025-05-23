package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.AutoTargetUtils.Reef.AlgaeReefLocation;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import me.nabdev.oxconfig.ConfigurableClass;
import me.nabdev.oxconfig.ConfigurableClassParam;
import me.nabdev.oxconfig.OxConfig;
import me.nabdev.oxconfig.sampleClasses.ConfigurableLinearInterpolation;

public class Elevator extends SubsystemBase implements ConfigurableClass {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private final ConfigurableLinearInterpolation L1 = new ConfigurableLinearInterpolation("Elevator L1 Heights");
    private final ConfigurableLinearInterpolation L2 = new ConfigurableLinearInterpolation("Elevator L2 Heights");
    private final ConfigurableLinearInterpolation L3 = new ConfigurableLinearInterpolation("Elevator L3 Heights");
    private final ConfigurableLinearInterpolation L4 = new ConfigurableLinearInterpolation("Elevator L4 Heights");

    private final ConfigurableLinearInterpolation intakeCoral = new ConfigurableLinearInterpolation(
            "Elevator Intake Heights");

    private final ConfigurableLinearInterpolation lowAlgae = new ConfigurableLinearInterpolation("Elevator Low Algae");
    private final ConfigurableLinearInterpolation highAlgae = new ConfigurableLinearInterpolation(
            "Elevator High Algae");

    private final ConfigurableClassParam<Double> elevatorTargetThreshold = new ConfigurableClassParam<>(this, 0.075,
            "Elevator Target Threshold");

    private final ConfigurableClassParam<Double> idleHeight = new ConfigurableClassParam<>(this, 0.5,
            "Elevator Idle Height");
    private final ConfigurableClassParam<Double> stagedL4 = new ConfigurableClassParam<>(this, 0.25,
            "Elevator Staged L4 Height");
    private final ConfigurableClassParam<Double> netHeight = new ConfigurableClassParam<>(this, 0.668, "Net Height");

    private final List<ConfigurableClassParam<?>> params = List.of(elevatorTargetThreshold, idleHeight, netHeight,
            stagedL4);

    private final BooleanSupplier shoulderInLowerDangerZone;
    private final BooleanSupplier shoulderInUpperDangerZone;

    public Elevator(ElevatorIO io, BooleanSupplier shoulderInLowerDangerZone,
            BooleanSupplier shoulderInUpperDangerZone) {
        this.io = io;
        this.shoulderInLowerDangerZone = shoulderInLowerDangerZone;
        this.shoulderInUpperDangerZone = shoulderInUpperDangerZone;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setVoltage(voltage.in(Volts)), null, this));

        OxConfig.registerConfigurableClass(this);
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
        io.updateInputs(inputs, shoulderInLowerDangerZone.getAsBoolean(), shoulderInUpperDangerZone.getAsBoolean());
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator at target", isAtTarget());
    }

    public Command move(DoubleSupplier speed) {
        return Commands.runEnd(() -> io.setSpeed(speed.getAsDouble()), () -> io.setSpeed(0.0), this)
                .withName("Elevator manual move");
    }

    public Command toPosition(DoubleSupplier height) {
        return new FunctionalCommand(io::resetPosControl, () -> io.setPosition(height.getAsDouble()),
                (c) -> io.setVoltage(0), () -> false, this).withName("Elevator to position");
    }

    public Command toPositionNoStop(DoubleSupplier height) {
        return Commands.startRun(io::resetPosControl, () -> io.setPosition(height.getAsDouble()), this)
                .withName("Elevator to position");
    }

    public Command toCoral(Supplier<CoralLevel> level) {
        return toPosition(() -> getHeightForCoral(level.get(), getCloseDistanceForCoral(level.get()), false))
                .withName("Elevator to CoralLevel");
    }

    public Command toCoral(Supplier<CoralLevel> level, DoubleSupplier robotDistance) {
        return toPosition(() -> getHeightForCoral(level.get(), robotDistance.getAsDouble(), false))
                .withName("Elevator to CoralLevel");
    }

    public Command toCoralNoStop(Supplier<CoralLevel> level, DoubleSupplier robotDistance) {
        return toPositionNoStop(() -> getHeightForCoral(level.get(), robotDistance.getAsDouble(), false))
                .withName("Elevator to CoralLevel");
    }

    public Command toNet() {
        return toPosition(netHeight::get).withName("Elevator to Net");
    }

    public Command intakeCoral(DoubleSupplier robotDistance) {
        return toPosition(() -> intakeCoral.calculate(robotDistance.getAsDouble()))
                .withName("Elevator to intake");
    }

    public Command intakeCoral() {
        return Commands.run(() -> io.setPosition(intakeCoral.calculate(intakeCoral.getX1())), this)
                .withName("Elevator to intake");
    }

    public Command stageIntake() {
        return toPosition(intakeCoral::getY2).withName("Elevator to intake");
    }

    public Command stageL4() {
        return toPosition(stagedL4::get).withName("Elevator to stage L4");
    }

    public Command idle() {
        return toPosition(idleHeight::get).withName("Elevator to idle");
    }

    public Command algaeIntake(Supplier<AlgaeReefLocation> location, DoubleSupplier distance) {
        return toPosition(() -> location.get().upperBranch() ? highAlgae.calculate(distance.getAsDouble())
                : lowAlgae.calculate(distance.getAsDouble())).withName("Elevator to algae intake");
    }

    public Command lowAlgae() {
        return toPosition(lowAlgae::getY1).withName("Elevator to low algae");
    }

    public Command highAlgae() {
        return toPosition(highAlgae::getY1).withName("Elevator to high algae");
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

    @AutoLogOutput(key = "IsAtTarget")
    public boolean isAtTarget() {
        return Math.abs(inputs.elevatorHeightMeters - inputs.setpointMeters) < elevatorTargetThreshold.get();
    }

    public boolean notAtTarget() {
        return !isAtTarget() && inPosControlMode();
    }

    public boolean inPosControlMode() {
        return inputs.inPosControlMode;
    }

    @Override
    public List<ConfigurableClassParam<?>> getParameters() {
        return params;
    }

    @Override
    public String getKey() {
        return "Elevator";
    }

    public void zero() {
        io.zero();
    }
}
