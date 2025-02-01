package frc.robot.subsystems.LEDs;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDsConstants {

    @AutoLog
    public static class LEDsIOInputs {
        public static int Length = 0;
        public static int PWM = 0;

    
    public static enum LedColor {
        Green(LEDPattern.solid(Color.kGreen)),
        Teal(LEDPattern.solid(Color.kTeal)),
        Orange(LEDPattern.solid(Color.kOrange)),
        White(LEDPattern.solid(Color.kWhite)),
        Blue(LEDPattern.solid(Color.kBlue));

        private LEDPattern solid;

        private LedColor(LEDPattern solid) {
            this.solid = solid;
        }
    };

    public static AddressableLEDBuffer buffer = new AddressableLEDBuffer(Length);
    public static AddressableLED LEDStrip = new AddressableLED(PWM);
    }
}