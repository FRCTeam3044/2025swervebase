package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.currentLimit;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.intakeMotorReduction;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.wristMotorReduction;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax algeaMotor = new SparkMax(algeaMotorCanId, MotorType.kBrushless);
    private final SparkMax shoulderMotor = new SparkMax(shoulderCanId, MotorType.kBrushless);
    private final SparkMax coralMotor = new SparkMax(coralMotorCanId, MotorType.kBrushless);
    private final AbsoluteEncoder shoulderEncoder = algeaMotor.getAbsoluteEncoder();
    
    public EndEffectorIOSpark() {
        var algeaMotorConfig = new SparkMaxConfig();
        algeaMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        algeaMotorConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / intakeMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / intakeMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(algeaMotor, 5, () -> algeaMotor.configure(algeaMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        
        var coralMotorConfig = new SparkMaxConfig();
        coralMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        coralMotorConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / intakeMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / intakeMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(coralMotor, 5, () -> coralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        

        var wristConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        wristConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / wristMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / wristMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(shoulderMotor, 5, () -> shoulderMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        ifOk(algeaMotor, shoulderEncoder::getVelocity, (value) -> inputs.intakeWheelsSpeedRadPerSec = value);
        ifOk(
            algeaMotor, 
            new DoubleSupplier[] {algeaMotor::getAppliedOutput, algeaMotor::getBusVoltage},
            (values) -> inputs.intakeWheelsAppliedVoltage = values[0] * values[1]);
        ifOk(algeaMotor, algeaMotor::getOutputCurrent, (value) -> inputs.intakeWheelsCurrantAmps = value);


        ifOk(shoulderMotor, shoulderEncoder::getPosition, (value) -> inputs.wristAngleRad = value);
        ifOk(shoulderMotor, shoulderEncoder::getVelocity, (value) -> inputs.wristSpeedRadsPerSec = value);
        ifOk(shoulderMotor, shoulderMotor::getOutputCurrent, (value) -> inputs.wristCurrantAmps = value);
        ifOk(
            shoulderMotor, 
            new DoubleSupplier[] {algeaMotor::getAppliedOutput, algeaMotor::getBusVoltage},
            (values) -> inputs.wristAppliedVoltage = values[0] * values[1]);

    }

    @Override
    public void setShoulderAngle(double desiredangle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngle'");
    }

    @Override
    public void setShoulderSpeed(double desiredspeed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }

    @Override
    public void setIntakeSpeed(double desiredspeed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }
}
