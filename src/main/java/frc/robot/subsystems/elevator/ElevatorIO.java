package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRad = 0.0;
        public double speedRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        

        public boolean topHallEffectClosed = false;
        public boolean bottomEffectClosed = false;
        public double velocityRadPerSec;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public void setPosition (double desiredPosition);
    public void setSpeed (double desiredSpeed);
}