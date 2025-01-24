package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkMax leftElevatorMotor = new SparkMax(leftCanId, MotorType.kBrushless);
    private final SparkMax rightElevatorMotor = new SparkMax(rightCanId, MotorType.kBrushless);

    private final RelativeEncoder elevatorEncoder = leftElevatorMotor.getEncoder();

    private final DigitalInput topHallEffect = new DigitalInput(0);
    private final DigitalInput bottomHallEffect = new DigitalInput(1);

    public ElevatorIOSpark() {

        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).encoder
                .positionConversionFactor(0);

        tryUntilOk(rightElevatorMotor, 5, () -> rightElevatorMotor.configure(configRight,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        SparkMaxConfig configLeft = new SparkMaxConfig();
        configLeft.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);

        tryUntilOk(leftElevatorMotor, 5, () -> leftElevatorMotor.configure(configLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

        configRight.follow(leftElevatorMotor);
    }

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

        if(bottomHallEffect.get()) {
            elevatorEncoder.setPosition(bottomBarMeters);
            
        }

        if(topHallEffect.get()) {
            //Unknown height
            elevatorEncoder.setPosition(topBarMeters);
        }
    }
}
