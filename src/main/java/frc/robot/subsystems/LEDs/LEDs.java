package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDs extends SubsystemBase {

    public Command Default(){
        return Commands.run(LEDsIO.LEDsIOInputs.yellow.applyTo(LEDsConstants.LEDStrip), //add if normal);
    }

    public Command hasCoral(){
        return null;
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
