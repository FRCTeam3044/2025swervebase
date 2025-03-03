package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double leaderPositionRot = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double leaderVelocityRPM = 0.0;
        public double leaderTemperature = 0.0;
        public double followerTemperature = 0.0;

        public double setpointRotations = 0.0;
        public double setpointMeters = 0.0;
        public double elevatorHeightMeters = 0.0;

        public boolean topHallEffectClosed = false;
        public boolean bottomEffectClosed = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs, boolean shoulderInDangerZone) {
    };

    public default void setPosition(double desiredPosition) {
    };

    public default void setSpeed(double desiredSpeed) {
    };

    public default void setVoltage(double voltage) {
    };

    public default void resetPosControl() {

    }

    public default void zero() {

    }
}