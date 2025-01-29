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
    

    public EndEffectorIOSpark() {
        var algaeMotorConfig = new SparkMaxConfig();
        algaeMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        algaeMotorConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / algaeMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / algaeMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

       

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
      
        ifOk(
                coralMotor,
                new DoubleSupplier[] { coralMotor::getAppliedOutput, coralMotor::getBusVoltage },
                (values) -> inputs.coralAppliedVoltage = values[0] * values[1]);
       
    }

    @Override
    public void setCoralSpeed(double speed) {
        coralMotor.set(speed);
    }
}
