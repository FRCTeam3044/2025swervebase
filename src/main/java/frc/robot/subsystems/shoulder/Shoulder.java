package frc.robot.subsystems.shoulder;

import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.ConfigurableLinearInterpolation;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;

public class Shoulder extends SubsystemBase {
    private final ShoulderIO io;
    private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private final ConfigurableLinearInterpolation L1 = new ConfigurableLinearInterpolation("Shoulder L1 Angles");
    private final ConfigurableLinearInterpolation L2 = new ConfigurableLinearInterpolation("Shoulder L2 Angles");
    private final ConfigurableLinearInterpolation L3 = new ConfigurableLinearInterpolation("Shoulder L3 Angles");
    private final ConfigurableLinearInterpolation L4 = new ConfigurableLinearInterpolation("Shoulder L4 Angles");

    private final ConfigurableLinearInterpolation intakeCoral = new ConfigurableLinearInterpolation(
            "Shoulder Intake Angles");

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
                        return inputs.leftShoulderAngleRad > 1.4 * Math.PI;
                    } else {
                        return inputs.leftShoulderAngleRad < -Math.PI / 2.1;
                    }
                });
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction)).until(() -> {
            if (direction == SysIdRoutine.Direction.kForward) {
                return inputs.leftShoulderAngleRad > 1.4 * Math.PI;
            } else {
                return inputs.leftShoulderAngleRad < -Math.PI / 2.1;
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

    public Command scoreCoral(CoralLevel level, DoubleSupplier robotDistance) {
        return Commands
                .run(() -> io.setShoulderAngle(calculateAngleForCoral(level, robotDistance.getAsDouble())), this)
                .withName("Shoulder to CoralLevel");
    }

    public Command intakeCoral(DoubleSupplier robotDistance) {
        return Commands.run(() -> io.setShoulderAngle(intakeCoral.calculate(robotDistance.getAsDouble())), this)
                .withName("Shoulder to Intake");
    }

    private double calculateAngleForCoral(CoralLevel level, double robotDist) {
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

    public double getShoulderAngle() {
        return inputs.leftShoulderAngleRad;
    }
}
