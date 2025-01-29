package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class LEDsIO {
    public static class LEDsIOInputs {
        public static int Length = 0;
        public static double PWM = 0.0;
    }

    public static enum Colors {
        Green(LEDPattern.solid(Color.kGreen));

        private LEDPattern pattern;
        
        Colors(LEDPattern pattern) {
            this.pattern = pattern;
        }
    };

    public void defult(){};
    public void hasCoral(){};
    public void hasAlgea(){};
    public void goingToCoralIntake(){};
    public void goingToAlgaeIntake(){};
    public void intakingCoral(){};
    public void scoringCoral(){};
    public void scoringAlgae(){};
}
