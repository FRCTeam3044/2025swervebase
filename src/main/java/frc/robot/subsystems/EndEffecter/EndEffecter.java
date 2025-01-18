package frc.robot.subsystems.EndEffecter;

import edu.wpi.first.wpilibj2.command.Command;

public class EndEffecter {
    private final EndEffecterIO io;
    private final EndEffecterIOInputsAutoLogged inputs = new EndEffecterIOInputsAutoLogged();

    public EndEffecter(EndEffecterIO io) {
        this.io = io;
    }

    public Command pivot(double desiredangle) {
        return null;
    }

    public Command manualpivot(double desiredspeed) {
        return null;
    }

    public Command scoreL1() {
        return null;
    }

    public Command scoreL2AndL3() {
        return null;
    }

    public Command scoreL4() {
        return null;
    }

}
