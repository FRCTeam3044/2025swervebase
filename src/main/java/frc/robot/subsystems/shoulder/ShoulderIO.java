package frc.robot.subsystems.shoulder;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {

    @AutoLog
    public static class ShoulderIOInputs {
        public double shoulderAngleRad = 0.0;
        public double shoulderSpeedRadsPerSec = 0.0;
        public double shoulderAppliedVoltage = 0.0;
        public double shoulderCurrantAmps = 0.0;
    }

    public void updateInputs(ShoulderIOInputs inputs);

    public void setShoulderAngle(double angle);

    public void setShoulderSpeed(DoubleSupplier speed);
}