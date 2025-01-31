package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double coralAppliedVoltage = 0.0;
        public double coralCurrentAmps = 0.0;

        public double proximitySensorDistance = 0.0;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {
    };

    public default void setCoralSpeed(double speed) {
    };
}
