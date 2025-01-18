package frc.robot.subsystems.EndEffecter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffecter extends SubsystemBase{
    private final EndEffecterIO io;
    private final EndEffecterIOInputsAutoLogged inputs = new EndEffecterIOInputsAutoLogged();

    enum LevelAngle {
        L1(0, 0, 0, 0),
        L23(0,0,0,0),
        L4(0,0,0,0);

        private double closeDist;
        private double closeAngle;
        private double farDist;
        private double farAngle;

        private LevelAngle (double closeDist, double closeAngle, double farDist, double farAngle){
            this.closeAngle = closeAngle;
            this.closeDist = closeDist;
            this.farDist = farDist;
            this.farAngle = farAngle;
        }
    };



    public EndEffecter(EndEffecterIO io) {
        this.io = io;
    }

    public Command pivot(LevelAngle levelAngles) {
        return null;
    }

    public Command manualpivot(double desiredspeed) {
        return null;
    }

    public Command scoreL1() {
        return Commands.run(() -> pivot(levelAngles.L1) , this).withName("Set wrist to L1 Scoring position");
    }

    public Command scoreL2AndL3() {
        return null;
    }

    public Command scoreL4() {
        return null;
    }

    private double calculateAngleForDist(double robotDist, LevelAngle desiredLevel){
        
    }

}
