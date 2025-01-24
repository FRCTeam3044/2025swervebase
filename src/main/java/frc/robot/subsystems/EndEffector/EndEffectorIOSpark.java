package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax coralMotor = new SparkMax(coralCanId, MotorType.kBrushless);
    private final SparkMax algaeMotor = new SparkMax(algaeCanId, MotorType.kBrushless);

    public EndEffectorIOSpark() {
        var algaeMotorConfig = new SparkMaxConfig();
        algaeMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        algaeMotorConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / algaeMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / algaeMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(algaeMotor, 5, () -> algaeMotor.configure(algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        var coralMotorConfig = new SparkMaxConfig();
        coralMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        coralMotorConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / coralMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / coralMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(coralMotor, 5, () -> coralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        ifOk(coralMotor, coralMotor::getOutputCurrent, (value) -> inputs.coralCurrentAmps = value);
        ifOk(algaeMotor, algaeMotor::getOutputCurrent, (value) -> inputs.algaeCurrentAmps = value);
        ifOk(
                coralMotor,
                new DoubleSupplier[] { coralMotor::getAppliedOutput, coralMotor::getBusVoltage },
                (values) -> inputs.coralAppliedVoltage = values[0] * values[1]);
        ifOk(
                algaeMotor,
                new DoubleSupplier[] { algaeMotor::getAppliedOutput, algaeMotor::getBusVoltage },
                (values) -> inputs.algaeAppliedVoltage = values[0] * values[1]);
    }

    @Override
    public void setCoralSpeed(double speed) {
        coralMotor.set(speed);
    }

    @Override
    public void setAlgaeSpeed(double speed) {
        algaeMotor.set(speed);
    }
}
