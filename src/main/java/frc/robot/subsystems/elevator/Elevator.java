package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    enum LevelHeight {
        L1(0, 0, 0, 0),
        L2(0,0,0,0),
        L3(0,0,0,0),
        L4(0,0,0,0);

        private double closeDist;
        private double closeHeight;
        private double farDist;
        private double farHeight;

        private LevelHeight (double closeDist, double closeHeight, double farDist, double farHeight){
            this.closeHeight = closeHeight;
            this.closeDist = closeDist;
            this.farDist = farDist;
            this.farHeight = farHeight;
        }
    };

    public Elevator(ElevatorIO io) {
        this.io = io;
    }
    public Command elevatorMove(double speed){
        return null;
    }

    public Command runOnJoystick(){
        return null;
    }

    public Command toL1(DoubleSupplier robotDistance){
        return Commands.run(() -> io.setHeight(calculateAngleForDist(robotDistance.getAsDouble(), LevelHeight.L1), this)).withName("Set elevator to L1 Scoring level");
    }

    public Command toL2(DoubleSupplier robotDistance){
        return Commands.run(() -> io.setHeight(calculateAngleForDist(robotDistance.getAsDouble(), LevelHeight.L2), this)).withName("Set elevator to L2 Scoring level");
    }
    
    public Command toL3(DoubleSupplier robotDistance){
        return Commands.run(() -> io.setHeight(calculateAngleForDist(robotDistance.getAsDouble(), LevelHeight.L4), this)).withName("Set elevator to L3 Scoring level");
    }

    public Command toL4(DoubleSupplier robotDistance){
        return Commands.run(() -> io.setHeight(calculateAngleForDist(robotDistance.getAsDouble(), LevelHeight.L4), this)).withName("Set elevator to L4 Scoring level");
    }

    public Command intake(){
        return null;
    } 

    private double calculateAngleForDist(double robotDist, LevelHeight desiredLevel){
        double heightForDist = ((desiredLevel.farHeight-desiredLevel.closeHeight) / (desiredLevel.farDist-desiredLevel.closeDist)) * (robotDist - desiredLevel.closeHeight);
        return heightForDist;
    }
}
