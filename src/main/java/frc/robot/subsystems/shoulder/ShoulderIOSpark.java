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
    private final SparkMax shoulderMotor = new SparkMax(shoulderCanId, MotorType.kBrushless);
    private final AbsoluteEncoder shoulderEncoder = shoulderMotor.getAbsoluteEncoder();
    
    public ShoulderIOSpark() {
        var shoulderMotorConfig = new SparkMaxConfig();
        shoulderMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        shoulderMotorConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / shoulderMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / shoulderMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(shoulderMotor, 5, () -> shoulderMotor.configure(shoulderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        ifOk(shoulderMotor, shoulderEncoder::getPosition, (value) -> inputs.shoulderAngleRad = value);
        ifOk(shoulderMotor, shoulderEncoder::getVelocity, (value) -> inputs.shoulderSpeedRadsPerSec = value);
        ifOk(shoulderMotor, shoulderMotor::getOutputCurrent, (value) -> inputs.shoulderCurrantAmps = value);
        ifOk(
                shoulderMotor,
                new DoubleSupplier[] { shoulderMotor::getAppliedOutput, shoulderMotor::getBusVoltage },
                (values) -> inputs.shoulderAppliedVoltage = values[0] * values[1]);
    }

    @Override
    public void setShoulderAngle(double desiredangle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngle'");
    }

    @Override
    public void setShoulderSpeed(DoubleSupplier desiredSpeed) {
        // TODO Auto-generated method stub
        shoulderMotor.set(desiredSpeed.getAsDouble());
    }
}
