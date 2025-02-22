package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.ToMorseCode;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.LEDs.LEDsConstants.*;

import java.io.Reader;
import java.util.Map;

public class LEDsIORio implements LEDsIO {
    private final AddressableLED LEDStrip = new AddressableLED(PWM);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);

    public LEDsIORio() {
        LEDStrip.setLength(buffer.getLength());
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
        LEDPattern pattern = step.scrollAtRelativeSpeed(Percent.per(Seconds).of(25));
        pattern.applyTo(buffer);
        LEDStrip.setData(buffer);
    }

    @Override
    public void makeMorseCode(String phrase) {
        if (ToMorseCode.toMorseCode(phrase).get(LEDsIOInputs.indexOfStr).toString()
                .charAt(LEDsIOInputs.indexOfChar) == '.') {
            LEDsIOInputs.currentTime = Seconds.of(Timer.getFPGATimestamp());
            setSolidColor(LEDPattern.solid(Color.kYellow));
            if (LEDsIOInputs.timeOfDot.plus(LEDsIOInputs.currentTime).lt(Seconds.of(Timer.getFPGATimestamp()))) {
                if (ToMorseCode.toMorseCode(phrase).get(LEDsIOInputs.indexOfStr).toString()
                        .length() < LEDsIOInputs.indexOfChar) {
                    LEDsIOInputs.indexOfChar = 0;
                    LEDsIOInputs.indexOfStr++;
                } else {
                    LEDsIOInputs.indexOfChar++;
                }
                setSolidColor(LEDPattern.solid(Color.kPurple));
            }
        } else {
            LEDsIOInputs.currentTime = Seconds.of(Timer.getFPGATimestamp());
            setSolidColor(LEDPattern.solid(Color.kYellow));
            if (LEDsIOInputs.currentTime.plus(LEDsIOInputs.timeOfDash)
                    .lt(Seconds.of(Timer.getFPGATimestamp()))) {
                if (ToMorseCode.toMorseCode(phrase).get(LEDsIOInputs.indexOfStr).toString()
                        .length() < LEDsIOInputs.indexOfChar) {
                    LEDsIOInputs.indexOfChar = 0;
                    LEDsIOInputs.indexOfStr++;
                } else {
                    LEDsIOInputs.indexOfChar++;
                }
                setSolidColor(LEDPattern.solid(Color.kPurple));
            }
        }
        if(phrase.charAt(LEDsIOInputs.indexOfStr) == ' ') {
            setSolidColor(LEDPattern.solid(Color.kYellow));
            LEDsIOInputs.currentTime = Seconds.of(Timer.getFPGATimestamp());
            if (LEDsIOInputs.currentTime.plus(LEDsIOInputs.timeOfDot)
            .plus(LEDsIOInputs.timeOfDot).plus(LEDsIOInputs.timeOfDot)
            .plus(LEDsIOInputs.timeOfDot) .lt(Seconds.of(Timer.getFPGATimestamp()))) {
                setSolidColor(LEDPattern.solid(Color.kPurple));
            }
        }
    }
}
