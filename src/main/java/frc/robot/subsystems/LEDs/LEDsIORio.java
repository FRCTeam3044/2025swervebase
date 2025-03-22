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
import frc.robot.util.AllianceUtil;
import frc.robot.util.ToMorseCode;
import frc.robot.util.AllianceUtil.AllianceColor;

public class LEDsIORio implements LEDsIO {
    private final AddressableLED LEDStrip = new AddressableLED(PWM);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);

    private final double blinkSpeed = 0.25;
    private final double spinSpeed = 30;

    public LEDsIORio() {
        LEDStrip.setLength(buffer.getLength());
        LEDStrip.setColorOrder(ColorOrder.kRGB);
        LEDStrip.start();
    }

    public void setSolidColor(LEDPattern color) {
        color.applyTo(buffer);
        LEDStrip.setData(buffer);
    }

    @Override
    public void setSeesAprilTag(boolean coral) {
        Color color = AllianceUtil.getAlliance() == AllianceColor.RED ? Color.kRed : Color.kBlue;
        if (AllianceUtil.getAlliance() == AllianceColor.UNKNOWN) {
            color = Color.kPurple;
        }

        LEDPattern.solid(Color.kGreen).applyTo(buffer);

        if (coral) {
            for (int i = 0; i < buffer.getLength(); i++) {
                if (i % 5 == 0 || i % 5 == 1) {
                    buffer.setLED(i, color);
                }
            }
        }
        LEDStrip.setData(buffer);
    }

    private int leftPatternPosition = 0;
    private int rightPatternPosition = 0;
    private int updateCounter = 0;
    private final int UPDATE_FREQUENCY = 8; // Only update position every 3 calls (60ms)

    @Override
    public void setSolidColorWithAuto(LEDPattern color) {
        // First apply the base pattern
        color.applyTo(buffer);

        // Then overlay the converging pattern
        overlayConvergingPattern();

        LEDStrip.setData(buffer);
    }

    @Override
    public void setOff() {
        // First apply the base pattern
        LEDPattern.kOff.applyTo(buffer);

        LEDStrip.setData(buffer);
    }

    @Override
    public void setBlinkingColorWithAuto(Color color) {
        // Create and apply the base blinking pattern
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern pattern = base.blink(Seconds.of(blinkSpeed));
        pattern.applyTo(buffer);

        // Then overlay the converging pattern
        overlayConvergingPattern();

        LEDStrip.setData(buffer);
    }

    // Helper method to overlay the converging pattern
    private void overlayConvergingPattern() {
        Color color = AllianceUtil.getAlliance() == AllianceColor.RED ? Color.kRed : Color.kBlue;
        // Slow down the animation by only updating every few calls
        updateCounter++;
        if (updateCounter >= UPDATE_FREQUENCY) {
            updateCounter = 0;

            // Update positions for next frame - move just one position at a time
            leftPatternPosition = (leftPatternPosition + 1) % 5; // Keep within pattern length
            rightPatternPosition = (rightPatternPosition - 1 + 5) % 5; // Keep within pattern length
        }

        // Draw the complete repeating pattern
        for (int i = 0; i < buffer.getLength(); i++) {
            // For left side pattern (moving right)
            if (i <= buffer.getLength() / 2 && (i % 5 == leftPatternPosition || i % 5 == (leftPatternPosition - 1))) {
                buffer.setLED(i, color);
            }

            // For right side pattern (moving left)
            // This creates a separate pattern from the right side
            // that moves in the opposite direction
            if (i >= buffer.getLength() / 2 && (i % 5 == rightPatternPosition || i % 5 == (leftPatternPosition - 1))) {
                buffer.setLED(i, color);
            }
        }
    }

    @Override
    public void setBlinkingColor(Color color) {
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern pattern = base.blink(Seconds.of(blinkSpeed));
        pattern.applyTo(buffer);
        LEDStrip.setData(buffer);
    }

    @Override
    public void setSpinningColor(Color color1, Color color2) {
        LEDPattern step = LEDPattern.steps(Map.of(0, color1, 0.46, color2, 0.5, color1, 0.96, color2));
        LEDPattern pattern = step.scrollAtRelativeSpeed(Percent.per(Seconds).of(spinSpeed));
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
