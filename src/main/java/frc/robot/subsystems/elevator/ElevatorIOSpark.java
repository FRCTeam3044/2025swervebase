package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import org.ejml.dense.row.linsol.qr.LinearSolverQrHouseCol_MT_FDRM;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkMax leftElevatorMotor = new SparkMax(leftCanId, null);
    private final SparkMax rightElevatorMotor = new SparkMax(rightCanId, null);

    private final DigitalInput topHallEffect = new DigitalInput(0);
    private final DigitalInput bottomHallEffect = new DigitalInput(0);

    public ElevatorIOSpark(int currentLimit) {

        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(0);
        configRight.encoder
                .positionConversionFactor(0);

        tryUntilOk(rightElevatorMotor, 5, () -> rightElevatorMotor.configure(configRight,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        SparkMaxConfig configLeft = new SparkMaxConfig();
        configLeft.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(0).inverted(true);
        configLeft.encoder
                .positionConversionFactor(0);

        tryUntilOk(leftElevatorMotor, 5, () -> leftElevatorMotor.configure(configLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

        configRight.follow(leftElevatorMotor);
    }

    private final RelativeEncoder elevatorEncoder = leftElevatorMotor.getEncoder();

    @Override
    public void setPosition(double desiredPosition) {
        leftElevatorMotor.getClosedLoopController().setReference(desiredPosition, ControlType.kCurrent);
    }

    @Override
    public void setSpeed(double desiredSpeed) {
        leftElevatorMotor.set(desiredSpeed);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        ifOk(leftElevatorMotor, elevatorEncoder::getPosition, (value) -> inputs.positionRad = value);
        ifOk(leftElevatorMotor, elevatorEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
                leftElevatorMotor,
                new DoubleSupplier[] { leftElevatorMotor::getAppliedOutput, leftElevatorMotor::getBusVoltage },
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(leftElevatorMotor, leftElevatorMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);

        inputs.bottomEffectClosed = bottomHallEffect.get();
        inputs.topHallEffectClosed = topHallEffect.get();
    }
}
