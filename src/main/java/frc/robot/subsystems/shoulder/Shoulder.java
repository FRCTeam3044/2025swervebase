package frc.robot.subsystems.shoulder;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private final List<ConfigurableClassParam<?>> params = List.of(stagingIntake, threshold,
            idle, dangerZoneOneMin, dangerZoneOneMax, dangerZoneTwoMin, dangerZoneTwoMax, stagingHighAlgae,
            stagingLowAlgae);

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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction))
                .until(() -> {
                    if (direction == SysIdRoutine.Direction.kForward) {
                        return inputs.leaderShoulderAngleRad > 3.8;
                    } else {
                        return inputs.leaderShoulderAngleRad < 0.51;
                    }
                });
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction)).until(() -> {
            if (direction == SysIdRoutine.Direction.kForward) {
                return inputs.leaderShoulderAngleRad > 3.8;
            } else {
                return inputs.leaderShoulderAngleRad < 0.51;
            }
        });

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shoulder", inputs);
        Logger.recordOutput("ShoulderInDangerZone", inDangerZone());
    }

    public Command toPosition(DoubleSupplier positiin) {
        return Commands.run(() -> io.setShoulderAngle(positiin.getAsDouble()), this)
                .withName("Shoulder to Position");
    }

    public Command manualPivot(DoubleSupplier desiredSpeed) {
        return Commands
                .runEnd(() -> io.setShoulderSpeed(desiredSpeed.getAsDouble()), () -> io.setShoulderSpeed(0.0), this)
                .withName("Shoulder Manual Pivot");
    }

    public Command scoreCoral(Supplier<CoralLevel> level, DoubleSupplier robotDist, BooleanSupplier staging) {
        return Commands
                .run(() -> io.setShoulderAngle(
                        calculateAngleForCoral(level.get(), robotDist.getAsDouble(), staging.getAsBoolean())), this)
                .withName("Shoulder to CoralLevel");
    }

    public Command scoreCoral(Supplier<CoralLevel> level) {
        return Commands
                .run(() -> io.setShoulderAngle(
                        calculateAngleForCoral(level.get(), getCloseCoralDistance(level.get()), false)), this)
                .withName("Shoulder to CoralLevel");
    }

    public Command stageCoral(CoralLevel level) {
        return Commands
                .run(() -> io.setShoulderAngle(calculateAngleForCoral(level, 0.0, true)), this)
                .withName("Shoulder to CoralLevel");
    }

    public Command intakeCoral(DoubleSupplier robotDist, BooleanSupplier staging) {
        return Commands.run(() -> io.setShoulderAngle(staging.getAsBoolean() ? stagingIntake.get()
                : intakeCoral.calculate(robotDist.getAsDouble())), this)
                .withName("Shoulder to Intake");
    }

    public Command algaeIntake(Supplier<AlgaeReefLocation> algae, DoubleSupplier robotAngle, BooleanSupplier staging) {
        return Commands.run(() -> {
            double target;
            if (algae.get().upperBranch()) {
                target = staging.getAsBoolean() ? stagingHighAlgae.get()
                        : highAlgae.calculate(robotAngle.getAsDouble());
            } else {
                target = staging.getAsBoolean() ? stagingLowAlgae.get() : lowAlgae.calculate(robotAngle.getAsDouble());
            }
            io.setShoulderAngle(target);
        }).withName("Shoulder to Algae");
    }

    public Command intakeCoral() {
        return Commands.run(() -> io.setShoulderAngle(intakeCoral.getY1()), this)
                .withName("Shoulder to Intake");
    }

    public Command stageIntake() {
        return Commands.run(() -> io.setShoulderAngle(stagingIntake.get()), this).withName("Shoulder to Intake");
    }

    public Command idle() {
        return Commands.run(() -> io.setShoulderAngle(idle.get()), this).withName("Shoulder Idle");
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

    public boolean isAtCoralTarget(Supplier<CoralLevel> level, DoubleSupplier robotDist) {
        return Math.abs(inputs.leaderShoulderAngleRad
                - calculateAngleForCoral(level.get(), robotDist.getAsDouble(), false)) < threshold.get();
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
}
