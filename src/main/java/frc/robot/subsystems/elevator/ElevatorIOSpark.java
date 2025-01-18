package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkMax leftElevatorMoter = new SparkMax(leftCanId, null);
    private final SparkMax rightElevatorMoter = new SparkMax(rightCanId, null);
    
    private final DigitalInput topHallEffect = new DigitalInput(0);
    private final DigitalInput bottomHallEffect = new DigitalInput(0);



    public ElevatorIOSpark(int currentLimit){

        
        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(0);
        configRight
            .encoder 
            .positionConversionFactor(0);
        
        tryUntilOk(rightElevatorMoter, 5, () -> rightElevatorMoter.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
        SparkMaxConfig configLeft = new SparkMaxConfig();
        configLeft.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(0).inverted(true);
        configLeft
            .encoder
            .positionConversionFactor(0);
        
        tryUntilOk(leftElevatorMoter, 5, () -> leftElevatorMoter.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        configRight.follow(leftElevatorMoter);
    }

    
    private final RelativeEncoder elevatorEncoder = leftElevatorMoter.getEncoder();
    
    @Override
    public void setPosition(double desiredPosition) {
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }
    
    @Override
    public void setSpeed(double desiredSpeed) {
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }
    
    public void updateInputs(ElevatorIOInputs inputs) {
        ifOk(leftElevatorMoter, elevatorEncoder::getPosition, (value) ->inputs.positionRad = value);
        ifOk(leftElevatorMoter, elevatorEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
        leftElevatorMoter,
        new DoubleSupplier[] {leftElevatorMoter::getAppliedOutput, leftElevatorMoter::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(leftElevatorMoter, leftElevatorMoter::getOutputCurrent, (value) -> inputs.currentAmps = value);
        
        inputs.bottomEffectClosed = bottomHallEffect.get();
        inputs.topHallEffectClosed = topHallEffect.get();
    }
}
