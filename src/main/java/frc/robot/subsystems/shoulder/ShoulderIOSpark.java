package frc.robot.subsystems.shoulder;

import static frc.robot.subsystems.shoulder.ShoulderConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShoulderIOSpark implements ShoulderIO {
    private final SparkMax shoulderMotorOne = new SparkMax(shoulderCanId, MotorType.kBrushless);
    private final SparkMax shoulderMotorTwo = new SparkMax(shoulderCanId, null);
    private final AbsoluteEncoder shoulderEncoder = shoulderMotorOne.getAbsoluteEncoder();
    
    public ShoulderIOSpark() {
        var shoulderMotorOneConfig = new SparkMaxConfig();
        shoulderMotorOneConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        shoulderMotorOneConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / shoulderMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / shoulderMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(shoulderMotorOne, 5, () -> shoulderMotorOne.configure(shoulderMotorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        var shoulderMotorTwoConfig = new SparkMaxConfig();
        shoulderMotorTwoConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        shoulderMotorTwoConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / shoulderMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / shoulderMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        shoulderMotorTwoConfig.follow(shoulderMotorOne);
        tryUntilOk(shoulderMotorTwo, 5, () -> shoulderMotorTwo.configure(shoulderMotorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        ifOk(shoulderMotorOne, shoulderEncoder::getPosition, (value) -> inputs.shoulderAngleRad = value);
        ifOk(shoulderMotorOne, shoulderEncoder::getVelocity, (value) -> inputs.shoulderSpeedRadsPerSec = value);
        ifOk(shoulderMotorOne, shoulderMotorOne::getOutputCurrent, (value) -> inputs.shoulderCurrantAmps = value);
        ifOk(
                shoulderMotorOne,
                new DoubleSupplier[] { shoulderMotorOne::getAppliedOutput, shoulderMotorOne::getBusVoltage },
                (values) -> inputs.shoulderAppliedVoltage = values[0] * values[1]);
    }

    @Override
    public void setShoulderAngle(double desiredangle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngle'");
    }

    @Override
    public void setShoulderSpeed(double desiredSpeed) {
        // TODO Auto-generated method stub
        shoulderMotorOne.set(desiredSpeed);
    }
}
