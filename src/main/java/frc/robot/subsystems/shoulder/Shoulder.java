package frc.robot.subsystems.shoulder;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
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

public class Shoulder extends SubsystemBase implements ConfigurableClass {
    private final ShoulderIO io;
    private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private final ConfigurableLinearInterpolation L1 = new ConfigurableLinearInterpolation("Shoulder L1 Angles");
    private final ConfigurableClassParam<Double> stagingL1 = new ConfigurableClassParam<>(this, 0.0,
            "Staging L1 Angle");
    private final ConfigurableLinearInterpolation L2 = new ConfigurableLinearInterpolation("Shoulder L2 Angles");
    private final ConfigurableClassParam<Double> manualL2 = new ConfigurableClassParam<Double>(this, 0.96,
            "Shoulder Manual L2 Angle");
    private final ConfigurableClassParam<Double> processor = new ConfigurableClassParam<Double>(this, 0.5,
            "Shoulder Processor Angle");
    private final ConfigurableClassParam<Double> climber = new ConfigurableClassParam<Double>(this, 4.5,
            "Shoulder Climber Angle");

    private final ConfigurableClassParam<Double> stagingL2 = new ConfigurableClassParam<>(this, 0.0,
            "Staging L2 Angle");
    private final ConfigurableLinearInterpolation L3 = new ConfigurableLinearInterpolation("Shoulder L3 Angles");
    private final ConfigurableClassParam<Double> stagingL3 = new ConfigurableClassParam<>(this, 0.0,
            "Staging L3 Angle");
    private final ConfigurableLinearInterpolation L4 = new ConfigurableLinearInterpolation("Shoulder L4 Angles");
    private final ConfigurableClassParam<Double> stagingL4 = new ConfigurableClassParam<>(this, 0.0,
            "Staging L4 Angle");

    private final ConfigurableLinearInterpolation lowAlgae = new ConfigurableLinearInterpolation("Shoulder Low Algae");
    private final ConfigurableLinearInterpolation highAlgae = new ConfigurableLinearInterpolation(
            "Shoulder High Algae");
    private final ConfigurableClassParam<Double> stagingLowAlgae = new ConfigurableClassParam<>(this, 0.0,
            "Staging Low Algae Angle");
    private final ConfigurableClassParam<Double> stagingHighAlgae = new ConfigurableClassParam<>(this, 0.0,
            "Staging High Algae Angle");

    private final ConfigurableLinearInterpolation intakeCoral = new ConfigurableLinearInterpolation(
            "Shoulder Intake Angles");
    private final ConfigurableClassParam<Double> preNet = new ConfigurableClassParam<>(this, 0.9,
            "Pre-net angle");
    private final ConfigurableClassParam<Double> stagingIntake = new ConfigurableClassParam<>(this, 0.0,
            "Staging Intake Angle");

    private final ConfigurableClassParam<Double> threshold = new ConfigurableClassParam<>(this, 0.07,
            "Shoulder Target Threshold");

    private final ConfigurableClassParam<Double> idle = new ConfigurableClassParam<>(this, -Math.PI / 2,
            "Shoulder Idle Angle (rad)");

    private final ConfigurableClassParam<Double> dangerZoneOneMin = new ConfigurableClassParam<>(this, 0.0,
            "Danger Zone One Min Angle (rad)");
    private final ConfigurableClassParam<Double> dangerZoneOneMax = new ConfigurableClassParam<>(this, 0.0,
            "Danger Zone One Max Angle (rad)");
    private final ConfigurableClassParam<Double> dangerZoneTwoMin = new ConfigurableClassParam<>(this, 0.0,
            "Danger Zone Two Min Angle (rad)");
    private final ConfigurableClassParam<Double> dangerZoneTwoMax = new ConfigurableClassParam<>(this, 0.0,
            "Danger Zone Two Max Angle (rad)");

    private final ConfigurableClassParam<Double> slowSpinThreshold = new ConfigurableClassParam<>(this, 0.9,
            "Slow Spin Threshold (rad)");

    private final List<ConfigurableClassParam<?>> params = List.of(stagingIntake, threshold,
            idle, dangerZoneOneMin, dangerZoneOneMax, dangerZoneTwoMin, dangerZoneTwoMax, stagingHighAlgae,
            stagingLowAlgae, stagingL1, stagingL2, stagingL3, stagingL4, manualL2, processor, climber, preNet);

    private BooleanSupplier elevatorNotAtTarget;

    public Shoulder(ShoulderIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Shoulder/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
        OxConfig.registerConfigurableClass(this);
    }

    public void setElevatorNotAtTargetSupplier(BooleanSupplier elevatorNotAtTarget) {
        this.elevatorNotAtTarget = elevatorNotAtTarget;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction))
                .until(() -> {
                    if (direction == SysIdRoutine.Direction.kForward) {
                        return inputs.leaderShoulderAngleRad > 3.58;
                    } else {
                        return inputs.leaderShoulderAngleRad < 0.58;
                    }
                });
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction)).until(() -> {
            if (direction == SysIdRoutine.Direction.kForward) {
                return inputs.leaderShoulderAngleRad > 3.58;
            } else {
                return inputs.leaderShoulderAngleRad < 0.58;
            }
        });

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs, elevatorNotAtTarget.getAsBoolean());
        Logger.processInputs("Shoulder", inputs);
        Logger.recordOutput("ShoulderInDangerZone", inDangerZone());
    }

    public Command toPosition(DoubleSupplier position) {
        return new FunctionalCommand(() -> io.resetPosControl(), () -> io.setShoulderAngle(position.getAsDouble()),
                (b) -> io.setVoltage(0), () -> false, this)
                .withName("Shoulder to Position");
    }

    public Command toPositionNet(DoubleSupplier position) {
        return new FunctionalCommand(() -> io.resetPosControl(),
                () -> io.setShoulderAngle(position.getAsDouble(), true),
                (b) -> io.setVoltage(0), () -> false, this)
                .withName("Shoulder to Position (net speed)");
    }

    public Command toPositionNoStop(DoubleSupplier position) {
        return Commands.startRun(io::resetPosControl, () -> io.setShoulderAngle(position.getAsDouble()), this)
                .withName("Shoulder to Position");
    }

    public Command manualPivot(DoubleSupplier desiredSpeed) {
        return Commands
                .runEnd(() -> io.setShoulderSpeed(desiredSpeed.getAsDouble()), () -> io.setShoulderSpeed(0.0), this)
                .withName("Shoulder Manual Pivot");
    }

    public Command scoreCoral(Supplier<CoralLevel> level, DoubleSupplier robotDist, BooleanSupplier staging) {
        return toPosition(() -> calculateAngleForCoral(level.get(), robotDist.getAsDouble(), staging.getAsBoolean()))
                .withName("Shoulder to CoralLevel");
    }

    public Command scoreCoralNoStop(Supplier<CoralLevel> level, DoubleSupplier robotDist, BooleanSupplier staging) {
        return toPositionNoStop(
                () -> calculateAngleForCoral(level.get(), robotDist.getAsDouble(), staging.getAsBoolean()))
                .withName("Shoulder to CoralLevel");
    }

    public Command scoreCoral(Supplier<CoralLevel> level) {
        return toPosition(() -> calculateAngleForCoral(level.get(), getCloseCoralDistance(level.get()), false))
                .withName("Shoulder to CoralLevel");
    }

    public Command stageCoral(CoralLevel level) {
        return toPosition(() -> calculateAngleForCoral(level, 0.0, true))
                .withName("Shoulder to CoralLevel");
    }

    public Command intakeCoral(DoubleSupplier robotDist, BooleanSupplier staging) {
        return toPosition(
                () -> staging.getAsBoolean() ? stagingIntake.get()
                        : MathUtil.clamp(intakeCoral.calculate(robotDist.getAsDouble()), intakeCoral.getY2(),
                                intakeCoral.getY1()))
                .withName("Shoulder to Intake");
    }

    public Command algaeIntake(Supplier<AlgaeReefLocation> algae, DoubleSupplier robotAngle, BooleanSupplier staging) {
        return toPosition(() -> {
            double target;
            if (algae.get().upperBranch()) {
                target = staging.getAsBoolean() ? stagingHighAlgae.get()
                        : highAlgae.calculate(robotAngle.getAsDouble());
            } else {
                target = staging.getAsBoolean() ? stagingLowAlgae.get() : lowAlgae.calculate(robotAngle.getAsDouble());
            }
            return target;
        }).withName("Shoulder to Algae");
    }

    public Command highAlgae() {
        return toPosition(() -> highAlgae.getY1()).withName("Shoulder to high algae");
    }

    public Command lowAlgae() {
        return toPosition(() -> lowAlgae.getY1()).withName("Shoulder to low algae");
    }

    public Command intakeCoral() {
        return toPosition(intakeCoral::getY1).withName("Shoulder to Intake");
    }

    public Command preNet() {
        return toPosition(preNet::get).withName("Shoulder to Pre Net");
    }

    public Command stageIntake() {
        return toPosition(stagingIntake::get).withName("Shoulder to Intake");
    }

    public Command idle() {
        return toPosition(idle::get).withName("Shoulder Idle");
    }

    public Command processor() {
        return toPosition(processor::get).withName("Shoulder Processor");
    }

    public Command climb() {
        return toPosition(climber::get).withName("Shoulder Climb");
    }

    public Command net() {
        return toPositionNet(climber::get).withName("Shoulder Net");
    }

    private double calculateAngleForCoral(CoralLevel level, double robotDist, boolean staging) {
        if (staging)
            return getStagingAngle(level);
        switch (level) {
            case L1:
                return L1.calculate(robotDist);
            case L2:
                return L2.calculate(robotDist);
            case L3:
                return L3.calculate(robotDist);
            case L4:
                return L4.calculate(robotDist);
            default:
                return 0;
        }
    }

    private double getStagingAngle(CoralLevel level) {
        switch (level) {
            case L1:
                return stagingL1.get();
            case L2:
                return stagingL2.get();
            case L3:
                return stagingL3.get();
            case L4:
                return stagingL4.get();
            default:
                return 0;
        }
    }

    private double getCloseCoralDistance(CoralLevel level) {
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

    public double getShoulderAngle() {
        return inputs.leaderShoulderAngleRad;
    }

    public boolean isAtCoralStagingTarget(Supplier<CoralLevel> level) {
        return Math.abs(inputs.leaderShoulderAngleRad - getStagingAngle(level.get())) < threshold.get();
    }

    public boolean isAtAlgaeIntakeTarget(Supplier<AlgaeReefLocation> algae, DoubleSupplier robotAngle) {
        return Math.abs(inputs.leaderShoulderAngleRad
                - (algae.get().upperBranch() ? highAlgae.calculate(robotAngle.getAsDouble())
                        : lowAlgae.calculate(robotAngle.getAsDouble()))) < threshold.get();
    }

    private Debouncer atCoralDebouncer = new Debouncer(0.2);
    // private Debouncer atCoralFastDebouncer = new Debouncer(0.07);

    public boolean isAtCoralTarget(Supplier<CoralLevel> level, DoubleSupplier robotDist) {
        return atCoralDebouncer.calculate(Math.abs(inputs.leaderShoulderAngleRad
                - calculateAngleForCoral(level.get(), robotDist.getAsDouble(), false)) < threshold.get());
    }

    public boolean isAtCoralTargetFast(Supplier<CoralLevel> level, DoubleSupplier robotDist) {
        return Math.abs(inputs.leaderShoulderAngleRad
                - calculateAngleForCoral(level.get(), robotDist.getAsDouble(), false)) < threshold.get();
    }

    public boolean isAtProcessorTarget() {
        return Math.abs(inputs.leaderShoulderAngleRad - processor.get()) < threshold.get();
    }

    public boolean inDangerZone() {
        return (inputs.leaderShoulderAngleRad > dangerZoneOneMin.get()
                && inputs.leaderShoulderAngleRad < dangerZoneOneMax.get())
                || (inputs.leaderShoulderAngleRad > dangerZoneTwoMin.get()
                        && inputs.leaderShoulderAngleRad < dangerZoneTwoMax.get());
    }

    @Override
    public List<ConfigurableClassParam<?>> getParameters() {
        return params;
    }

    @Override
    public String getKey() {
        return "Shoulder";
    }

    @Override
    public String getPrettyName() {
        return "Shoulder";
    }

    public boolean inSafeZone() {
        return inputs.inSafeZone;
    }

    public boolean canSpinFast() {
        return inputs.leaderShoulderAngleRad < slowSpinThreshold.get();
    }
}
