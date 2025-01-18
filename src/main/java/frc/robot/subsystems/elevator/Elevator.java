package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io){
        this.io = io;
    }
    public Command elevatorMove(double speed){
        return null;
    }

    public Command runOnJoystick(){
        return null;
    }

    public Command toL1(){
        return null;
    }

    public Command toL2(){
        return null;
    }
    
    public Command toL3(){
        return null;
    }

    public Command toL4(){
        return null;
    }

    public Command intake(){
        return null;
    } 
}
