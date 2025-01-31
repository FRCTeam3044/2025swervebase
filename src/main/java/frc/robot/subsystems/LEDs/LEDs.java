/;package frc.robot.subsystems.LEDs;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class LEDs extends SubsystemBase {

    private final LEDsIO io;
    //I donno how to make this 
    private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

    public void setLEDS(LEDsConstants LEDStrips){
        LEDStrip.setLength((buffer.getLength()));
        LEDStrip.setData(buffer);
        LEDStrip.start();
    }


    public LEDs(LEDsIO io) {
        this.io = io;
    }

    public Command Default(){
        return null;
    }

    public Command hasCoral(LEDPattern color, LEDsConstants buffer){
        color.applyTo(buffer);
        return Commands.run(() -> LEDStrip.setData(buffer), this);
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
