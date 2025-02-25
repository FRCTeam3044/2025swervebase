package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;

        public double proximitySensorDistance = 0.0;

        public boolean hasCoral = false;
        public boolean hasAlgae = false;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {
    };

    public default void setSpeed(double speed) {
    };
}
