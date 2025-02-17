package frc.robot.subsystems.shoulder;

import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
    @AutoLog
    public static class ShoulderIOInputs {
        public double leftShoulderAngleRad = 0.0;
        public double leftShoulderRots = 0.0;
        public double leftShoulderSpeedRadsPerSec = 0.0;
        public double leftShoulderSpeedRPM = 0.0;
        public double leftShoulderAppliedVoltage = 0.0;
        public double leftShoulderCurrentAmps = 0.0;
        public double leftTemperature = 0.0;
        public double rightShoulderAngleRad = 0.0;
        public double rightShoulderSpeedRadsPerSec = 0.0;
        public double rightShoulderAppliedVoltage = 0.0;
        public double rightShoulderCurrentAmps = 0.0;
        public double rightTemperature = 0.0;
    }

    public default void updateInputs(ShoulderIOInputs inputs) {
    };

    public default void setShoulderAngle(double angle) {
    };

    public default void setShoulderSpeed(double speed) {
    };

    public default void setVoltage(double voltage) {
    };
}