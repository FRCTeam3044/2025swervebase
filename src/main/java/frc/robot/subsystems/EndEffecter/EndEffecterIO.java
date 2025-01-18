package frc.robot.subsystems.EndEffecter;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffecterIO {    

    @AutoLog
    public static class EndEffecterIOInputs{
        public double wristAngleRad = 0.0;
        public double wristSpeedRadsPerSec = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristCurrantAmps = 0.0; 
        public double intakeWheelsSpeedRadPerSec = 0.0;
        public double intakeWheelsAppliedVoltage = 0.0;
        public double intakeWheelsCurrantAmps = 0.0;
        
        public double proximitySensorDistance = 0.0;
    }

    public void updateInputs(EndEffecterIOInputs inputs);

    public void setAngle(double desiredangle);
    public void setSpeed(double desiredspeed);
}