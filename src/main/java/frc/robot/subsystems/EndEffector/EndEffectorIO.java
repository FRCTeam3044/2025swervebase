package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public static class EndEffectorIOInputs {
        public double wristAngleRad = 0.0;
        public double wristSpeedRadsPerSec = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristCurrantAmps = 0.0;
        public double intakeWheelsSpeedRadPerSec = 0.0;
        public double intakeWheelsAppliedVoltage = 0.0;
        public double intakeWheelsCurrantAmps = 0.0;

        public double proximitySensorDistance = 0.0;
    }

    public void updateInputs(EndEffectorIOInputs inputs);

    public void setWristAngle(double angle);

    public void setWristSpeed(double speed);

    public void setIntakeSpeed(double speed);
}