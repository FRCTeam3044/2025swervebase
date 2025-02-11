package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    enum LevelHeight {
        L1(0, 1, 0, 0),
        L2(0, 0, 0, 0),
        L3(0, 0, 0, 0),
        L4(0, 0, 0, 0);

        private double closeDist;
        private double closeHeight;
        private double farDist;
        private double farHeight;

        private LevelHeight(double closeDist, double closeHeight, double farDist, double farHeight) {
            this.closeHeight = closeHeight;
            this.closeDist = closeDist;
            this.farDist = farDist;
            this.farHeight = farHeight;
        }
    };

    public Elevator(ElevatorIO io) {
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
        Logger.processInputs("Elevator", inputs);
    }

    public Command elevatorMove(DoubleSupplier speed) {
        return Commands.runEnd(() -> io.setSpeed(speed.getAsDouble()), () -> io.setSpeed(0.0))
                .withName("Elevator manual move");
    }

    public Command toL1(DoubleSupplier robotDistance) {
        return Commands.run(() -> io.setPosition(calculateAngleForDist(robotDistance.getAsDouble(), LevelHeight.L1)))
                .withName("Set elevator to L1 Scoring level");
    }

    public Command toL2(DoubleSupplier robotDistance) {
        return Commands.run(() -> io.setPosition(calculateAngleForDist(robotDistance.getAsDouble(), LevelHeight.L2)))
                .withName("Set elevator to L2 Scoring level");
    }

    public Command toL3(DoubleSupplier robotDistance) {
        return Commands.run(() -> io.setPosition(calculateAngleForDist(robotDistance.getAsDouble(), LevelHeight.L4)))
                .withName("Set elevator to L3 Scoring level");
    }

    public Command toL4(DoubleSupplier robotDistance) {
        return Commands.run(() -> io.setPosition(calculateAngleForDist(robotDistance.getAsDouble(), LevelHeight.L4)))
                .withName("Set elevator to L4 Scoring level");
    }

    private double calculateAngleForDist(double robotDist, LevelHeight desiredLevel) {
        double heightForDist = ((desiredLevel.farHeight - desiredLevel.closeHeight)
                / (desiredLevel.farDist - desiredLevel.closeDist)) * (robotDist - desiredLevel.closeHeight);
        return heightForDist;
    }

    public ElevatorIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public double getElevatorHeight() {
        return inputs.elevatorHeightMeters;
    }
}
