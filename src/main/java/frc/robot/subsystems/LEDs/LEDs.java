package frc.robot.subsystems.LEDs;

import java.util.ArrayList;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ToMorseCode;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

public class LEDs extends SubsystemBase {

    private final LEDsIO io;

    public LEDs(LEDsIO io) {
        this.io = io;
    }

    public Command setColor(LEDPattern color) {
        return Commands.run(() -> io.setSolidColor(color), this).withName("Set LEDs");
    }

    public Command hasCoral() {
        // currentPattern = Color.kTeal;
        return setColor(LEDPattern.solid(Color.kTeal));
    }

    public Command hasAlgea() {
        return setColor(LEDPattern.solid(Color.kRed));
    }

    public Command goingToCoralIntake() {
        return setColor(LEDPattern.solid(Color.kOrange));
    }

    public Command goingToAlgaeIntake() {
        return setColor(LEDPattern.solid(Color.kGreen));
    }

    public Command aprilTagDetected() {
        return setColor(LEDPattern.solid(Color.kGreen));
    }

    public Command setBlinkingColor(Color color) {
        return Commands.run(() -> io.setBlinkingColor(color));
    }

    public Command intakingAndScoringCoral() {
        return setBlinkingColor(Color.kOrange);
    }

    public Command intakingAndScoringAlgae() {
        return setBlinkingColor(Color.kGreen);
    }

    public Command Default() {
        return Commands.run(() -> io.setSpinningColor(Color.kPurple, Color.kYellow));
    }

    public Command morseCode(String phrase) {
        setColor(LEDPattern.solid(Color.kPurple));
        int indexOfStr = 0;
        int indexOfChar = 0;
        Time timeOfDot = Milliseconds.of(500);
        Time timeOfDash = Milliseconds.of(1000);
        Time currentTime;
        return Command.runEnd(() -> {
        if(ToMorseCode.toMorseCode(phrase).toString().get(indexOfStr).charAt(indexOfChar) = '.') {
                currentTime = Seconds.of(edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
                setColor(LEDPattern.solid(Color.kYellow));
                if(currentTime + timeOfDot < Seconds.of(edu.wpi.first.wpilibj.Timer.getFPGATimestamp())) {
                    if(ToMorseCode.toMorseCode(phrase).get(indexOfStr).toString().length() < indexOfChar) {
                        indexOfChar = 0;
                        indexOfStr++;
                    }
                    else{
                        indexOfChar++;
                    }
                    break;
                }
            } else {
                currentTime = Seconds.of(edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
                setColor(LEDPattern.solid(Color.kYellow));
                if(currentTime + timeOfDash < Seconds.of(edu.wpi.first.wpilibj.Timer.getFPGATimestamp())) {
                    if(ToMorseCode.toMorseCode(phrase).get(indexOfStr).toString().length() < indexOfChar) {
                        indexOfChar = 0;
                        indexOfStr++;
                    }
                    else {
                        indexOfChar++;
                    }
                    break;
                }
            }}, indexOfStr > ToMorseCode.toMorseCode(phrase).size()).withName("Set Morse Code on LEDs");   
    }
}
