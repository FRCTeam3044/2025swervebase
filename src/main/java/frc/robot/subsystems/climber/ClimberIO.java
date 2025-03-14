package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public double servoPosition = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {
    }

    public default void setSpeed(double speed) {
    }

    public default void setServoClosed(boolean closed) {
    }
}
