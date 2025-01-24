package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double coralAppliedVoltage = 0.0;
        public double coralCurrentAmps = 0.0;
        public double algaeAppliedVoltage = 0.0;
        public double algaeCurrentAmps = 0.0;
        public double proximitySensorDistance = 0.0;
    }

    public void updateInputs(EndEffectorIOInputs inputs);

    public void setCoralSpeed(double speed);

    public void setAlgaeSpeed(double speed);
}
