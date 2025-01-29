package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDsIO {
    @AutoLog
    public static class LEDsIOInputs {
        public static int Length = 0;
        public static double PWM = 0.0;
    }

    public static enum LedColor {
        Green(LEDPattern.solid(Color.kGreen));

        private LEDPattern pattern;
        
        LedColor(LEDPattern pattern) {
            this.pattern = pattern;
        }
    };

    public void defaultLights(LedColor color){};
    public void hasCoral(){};
    public void hasAlgea(){};
    public void goingToCoralIntake(){};
    public void goingToAlgaeIntake(){};
    public void intakingCoral(){};
    public void scoringCoral(){};
    public void scoringAlgae(){};
}
