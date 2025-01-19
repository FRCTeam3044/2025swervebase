package frc.robot.subsystems.EndEffector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    enum LevelAngle {
        L1(0, 0, 0, 0),
        L23(0, 0, 0, 0),
        L4(0, 0, 0, 0);

        private double closeDist;
        private double closeAngle;
        private double farDist;
        private double farAngle;

        private LevelAngle(double closeDist, double closeAngle, double farDist, double farAngle) {
            this.closeAngle = closeAngle;
            this.closeDist = closeDist;
            this.farDist = farDist;
            this.farAngle = farAngle;
        }
    };

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }

    public Command manualpivot(double desiredspeed) {
        return null;
    }

    public Command scoreL1(DoubleSupplier robotDistance) {
        return Commands
                .run(() -> io.setWristAngle(calculateAngleForDist(robotDistance.getAsDouble(), LevelAngle.L1)), this)
                .withName("Set wrist to L1 Scoring position");
    }

    public Command scoreL2AndL3(DoubleSupplier robotDistance) {
        return Commands
                .run(() -> io.setWristAngle(calculateAngleForDist(robotDistance.getAsDouble(), LevelAngle.L23)), this)
                .withName("Set wrist to L1 Scoring position");
    }

    public Command scoreL4(DoubleSupplier robotDistance) {
        return Commands
                .run(() -> io.setWristAngle(calculateAngleForDist(robotDistance.getAsDouble(), LevelAngle.L4)), this)
                .withName("Set wrist to L1 Scoring position");
    }

    private double calculateAngleForDist(double robotDist, LevelAngle desiredLevel) {
        return ((desiredLevel.farAngle - desiredLevel.closeAngle)
                / (desiredLevel.farDist - desiredLevel.closeDist)) * (robotDist - desiredLevel.closeDist)
                + desiredLevel.closeAngle;
    }

}
