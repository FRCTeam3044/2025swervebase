package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.LEDs.LEDsConstants.PWM;
import static frc.robot.subsystems.LEDs.LEDsConstants.length;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.ToMorseCode;

public class LEDsIORio implements LEDsIO {
    private final AddressableLED LEDStrip = new AddressableLED(PWM);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);

    public LEDsIORio() {
        LEDStrip.setLength(buffer.getLength());
        LEDStrip.setColorOrder(ColorOrder.kRGB);
        LEDStrip.start();
    }

    @Override
    public void setSolidColor(LEDPattern color) {
        color.applyTo(buffer);
        LEDStrip.setData(buffer);
    }

    @Override
    public void setBlinkingColor(Color color) {
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern pattern = base.blink(Seconds.of(0.25));
        pattern.applyTo(buffer);
        LEDStrip.setData(buffer);
    }

    @Override
    public void setSpinningColor(Color color1, Color color2) {
        LEDPattern step = LEDPattern.steps(Map.of(0, color1, 0.46, color2, 0.5, color1, 0.96, color2));
        LEDPattern pattern = step.scrollAtRelativeSpeed(Percent.per(Seconds).of(20));
        pattern.applyTo(buffer);
        LEDStrip.setData(buffer);
    }

    @Override
    public void makeMorseCode(String phrase) {
        // Convert the phrase to Morse code once
        String currentSymbol = ToMorseCode.toMorseCode(phrase);

        // Iterate over each character in the Morse code string
        for (int i = 0; i < currentSymbol.length(); i++) {
            char currentChar = currentSymbol.charAt(i);

            // Handle dot (.)
            if (currentChar == '.') {
                setSolidColor(LEDPattern.solid(Color.kYellow)); // Turn LED ON for dot
                Timer.delay(0.2); // LED stays on for 0.2 seconds (adjust as needed)
                setSolidColor(LEDPattern.solid(Color.kPurple)); // Turn LED OFF after dot
                Timer.delay(0.2); // Pause between dots/dashes (adjust as needed)
            }
            // Handle dash (-)
            else if (currentChar == '-') {
                setSolidColor(LEDPattern.solid(Color.kYellow)); // Turn LED ON for dash
                Timer.delay(0.4); // LED stays on for 0.4 seconds (adjust as needed)
                setSolidColor(LEDPattern.solid(Color.kPurple)); // Turn LED OFF after dash
                Timer.delay(0.2); // Pause between dots/dashes (adjust as needed)
            }
            // Handle space (Pause between words)
            else if (currentChar == ' ') {
                Timer.delay(0.6); // Longer pause between words (adjust as needed)
            }
        }
    }
}
