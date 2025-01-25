package frc.robot.subsystems.shoulder;

import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shoulder extends SubsystemBase {
    private final ShoulderIO io;
    private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private final SysIdRoutine sysId;

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

    public Shoulder(ShoulderIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0))
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shoulder", inputs);
    }

    public Command manualPivot(DoubleSupplier desiredSpeed) {
        return Commands.runEnd(() -> io.setShoulderSpeed(desiredSpeed.getAsDouble()), () -> io.setShoulderSpeed(0.0));
    }

    public Command scoreL1(DoubleSupplier robotDistance) {
        return Commands
                .run(() -> io.setShoulderAngle(calculateAngleForDist(robotDistance.getAsDouble(), LevelAngle.L1)), this)
                .withName("Set Shoulder to L1 Scoring position");
    }

    public Command scoreL2AndL3(DoubleSupplier robotDistance) {
        return Commands
                .run(() -> io.setShoulderAngle(calculateAngleForDist(robotDistance.getAsDouble(), LevelAngle.L23)), this)
                .withName("Set Shoulder to L1 Scoring position");
    }

    public Command scoreL4(DoubleSupplier robotDistance) {
        return Commands
                .run(() -> io.setShoulderAngle(calculateAngleForDist(robotDistance.getAsDouble(), LevelAngle.L4)), this)
                .withName("Set Shoulder to L1 Scoring position");
    }

    private double calculateAngleForDist(double robotDist, LevelAngle desiredLevel) {
        return ((desiredLevel.farAngle - desiredLevel.closeAngle)
                / (desiredLevel.farDist - desiredLevel.closeDist)) * (robotDist - desiredLevel.closeDist)
                + desiredLevel.closeAngle;
    }

}
