package frc.robot.subsystems.shoulder;

import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
    @AutoLog
    public static class ShoulderIOInputs {
        public double leaderShoulderAngleRad = 0.0;
        public double leaderShoulderRots = 0.0;
        public double leaderShoulderSpeedRadsPerSec = 0.0;
        public double leaderShoulderSpeedRPM = 0.0;
        public double leaderShoulderAppliedVoltage = 0.0;
        public double leaderShoulderCurrentAmps = 0.0;
        public double leaderTemperature = 0.0;
        public double followerTemperature = 0.0;

        public double setpointAngleRad = 0.0;
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