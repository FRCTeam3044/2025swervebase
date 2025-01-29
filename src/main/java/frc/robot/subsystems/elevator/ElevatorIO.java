package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double leftPositionRad = 0.0;
        public double leftSpeedRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftTemperature = 0.0;
        public double rightPositionRad = 0.0;
        public double rightSpeedRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightTemperature = 0.0;
        public boolean topHallEffectClosed = false;
        public boolean bottomEffectClosed = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public void setPosition(double desiredPosition);

    public void setSpeed(double desiredSpeed);

    public void setVoltage(double voltage);
}