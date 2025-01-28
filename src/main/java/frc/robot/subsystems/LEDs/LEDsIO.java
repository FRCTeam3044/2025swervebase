package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDsIO {
    public static class LEDsIOInputs {
        public static int Length = 0;
        public static double PWM = 0.0;
        public static LEDPattern yellow= LEDPattern.solid(Color.kYellow);
    }

    public void defult(){};
    public void hasCoral(){};
    public void hasAlgea(){};
    public void goingToCoralIntake(){};
    public void goingToAlgaeIntake(){};
    public void intakingCoral(){};
    public void scoringCoral(){};
    public void scoringAlgae(){};
}
