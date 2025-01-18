package frc.robot.subsystems.EndEffecter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffecter extends SubsystemBase{
    private final EndEffecterIO io;
    private final EndEffecterIOInputsAutoLogged inputs = new EndEffecterIOInputsAutoLogged();

    enum levelAngles {
        L1,
        L2,
        L3,
        L4
    };



    public EndEffecter(EndEffecterIO io) {
        this.io = io;
    }

    public Command pivot(Enum levelAngles) {
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

}
