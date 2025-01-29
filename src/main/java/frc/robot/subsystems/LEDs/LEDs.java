package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDsIO.LEDsIOInputs.*;


public class LEDs extends SubsystemBase {

    private final LEDsIO io;
    private final LEDsIOInputAutoLogged inputs = new LEDsIOInputAutoLogged();

    public LEDs(LEDsIO io) {
        this.io = io;
    }

    public Command Default(){
        return null;
    }

    public Command hasCoral(){
        return Commands.run(LEDsIO.Colors.Green.applyTo(LEDsConstants.LEDStrip), this);
    }

    public Command hasAlgea(){
        return null;
    }

    public Command goingToCoralIntake(){
        return null;
    }

    public Command goingToAlgaeIntake(){
        return null;
    }

    public Command intakingCoral(){
        return null;
    }

    public Command scoringCoral(){
        return null;
    }

    public Command scoringAlgae(){
        return null;
    }
}
