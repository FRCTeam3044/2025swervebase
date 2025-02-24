package frc.robot.subsystems.shoulder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.ConfigurableLinearInterpolation;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import me.nabdev.oxconfig.ConfigurableParameter;

public class Shoulder extends SubsystemBase {
    private final ShoulderIO io;
    private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private final ConfigurableLinearInterpolation L1 = new ConfigurableLinearInterpolation("Shoulder L1 Angles");
    private final ConfigurableParameter<Double> stagingL1 = new ConfigurableParameter<>(0.0, "Staging L1 Angle");
    private final ConfigurableLinearInterpolation L2 = new ConfigurableLinearInterpolation("Shoulder L2 Angles");
    private final ConfigurableParameter<Double> stagingL2 = new ConfigurableParameter<>(0.0, "Staging L2 Angle");
    private final ConfigurableLinearInterpolation L3 = new ConfigurableLinearInterpolation("Shoulder L3 Angles");
    private final ConfigurableParameter<Double> stagingL3 = new ConfigurableParameter<>(0.0, "Staging L3 Angle");
    private final ConfigurableLinearInterpolation L4 = new ConfigurableLinearInterpolation("Shoulder L4 Angles");
    private final ConfigurableParameter<Double> stagingL4 = new ConfigurableParameter<>(0.0, "Staging L4 Angle");

    private final ConfigurableLinearInterpolation intakeCoral = new ConfigurableLinearInterpolation(
            "Shoulder Intake Angles");
    private final ConfigurableParameter<Double> stagingIntake = new ConfigurableParameter<>(0.0,
            "Staging Intake Angle");

    private final ConfigurableParameter<Double> threshold = new ConfigurableParameter<>(0.07,
            "Shoulder Target Threshold");

    private final ConfigurableParameter<Double> idle = new ConfigurableParameter<>(-Math.PI / 2,
            "Shoulder Idle Angle (rad)");

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
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction))
                .until(() -> {
                    if (direction == SysIdRoutine.Direction.kForward) {
                        return inputs.leaderShoulderAngleRad > 1.4 * Math.PI;
                    } else {
                        return inputs.leaderShoulderAngleRad < -Math.PI / 2.1;
                    }
                });
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction)).until(() -> {
            if (direction == SysIdRoutine.Direction.kForward) {
                return inputs.leaderShoulderAngleRad > 1.4 * Math.PI;
            } else {
                return inputs.leaderShoulderAngleRad < -Math.PI / 2.1;
            }
        });

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shoulder", inputs);
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

    public Command stageCoral(CoralLevel level) {
        return Commands
                .run(() -> io.setShoulderAngle(calculateAngleForCoral(level, 0.0, true)), this)
                .withName("Shoulder to CoralLevel");
    }

    public Command intakeCoral(DoubleSupplier robotAngle, BooleanSupplier staging) {
        return Commands.run(() -> io.setShoulderAngle(staging.getAsBoolean() ? stagingIntake.get()
                : intakeCoral.calculate(robotAngle.getAsDouble())), this)
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
}
