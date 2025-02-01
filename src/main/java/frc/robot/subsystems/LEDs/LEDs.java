package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

    private final LEDsIO io;

    public LEDs(LEDsIO io) {
        this.io = io;
    }

    public Command setColor(LEDPattern color) {
        return Commands.run(() -> io.setSolidColor(color), this);
    }

    public Command hasCoral(){
        return setColor(LEDPattern.solid(Color.kTeal));
    }

    public Command hasAlgea(){
        return setColor(LEDPattern.solid(Color.kRed));
    }

    public Command goingToCoralIntake(){
        return setColor(LEDPattern.solid(Color.kOrange));
    }

    public Command goingToAlgaeIntake(){
        return setColor(LEDPattern.solid(Color.kGreen));
    }

    public Command aprilTagDetected() {
        return setColor(LEDPattern.solid(Color.kGreen));
    }

    public Command intakingAndScoringCoral(){
        return Commands.none();
    }

    public Command intakingAndScoringAlgae(){
        return Commands.none();
    }

    public Command Default(){
        return Commands.none();
    }
}
