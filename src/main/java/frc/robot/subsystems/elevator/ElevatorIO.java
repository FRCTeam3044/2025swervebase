package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double leftPositionRot = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftVelocityRPM = 0.0;
        public double leftTemperature = 0.0;
        public double rightPositionRot = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightVelocityRPM = 0.0;
        public double rightTemperature = 0.0;

        public double setpointRotations = 0.0;
        public double setpointMeters = 0.0;
        public double elevatorHeightMeters = 0.0;

        public boolean topHallEffectClosed = false;
        public boolean bottomEffectClosed = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    };

    public default void setPosition(double desiredPosition) {
    };

    public default void setSpeed(double desiredSpeed) {
    };

    public default void setVoltage(double voltage) {
    };
}