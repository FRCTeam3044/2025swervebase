package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDsIO.LEDsIOInputs;
import frc.robot.util.ToMorseCode;

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
        return Commands.run(() -> io.setSpinningColor(Color.kPurple, Color.kYellow)).withName("Spinning LEDs");
    }

    public Command simMorseCode() {
        return morseCode("HI");
    }

    public Command morseCode(String phrase) {
        setColor(LEDPattern.solid(Color.kPurple));
        // return Commands.run(() -> {
        //     io.makeMorseCode(phrase);
        // })
        //         .until(() -> LEDsIOInputs.indexOfStr > ToMorseCode.toMorseCode(phrase).size())
        //         .withName("Make Morse Code");

        //return 
    }
}
