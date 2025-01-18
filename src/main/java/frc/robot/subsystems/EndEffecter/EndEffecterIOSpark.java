package frc.robot.subsystems.EndEffecter;

import static frc.robot.subsystems.EndEffecter.EndEffecterConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;

public class EndEffecterIOSpark implements EndEffecterIO {
    private final SparkMax intakeMotor = new SparkMax(rollerCanId, MotorType.kBrushless);
    private final SparkMax wristMotor = new SparkMax(wristCanId, MotorType.kBrushless);
    private final AbsoluteEncoder wristEncoder = intakeMotor.getAbsoluteEncoder();
    private final AnalogInput proximitySensor = new AnalogInput(proximitySensorChannel);
    
    public EndEffecterIOSpark() {
        var intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        intakeConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / intakeMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / intakeMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(intakeMotor, 5, () -> intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        
        var wristConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        wristConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / wristMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / wristMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(wristMotor, 5, () -> wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        
        PIDController wristPID = wristMotor.getPIDController();
        wristPID.setD(currentLimit);
        wristPID.setI(currentLimit);
        wristPID.setP(currentLimit);

    }

    @Override
    public void updateInputs(EndEffecterIOInputs inputs) {
        ifOk(intakeMotor, wristEncoder::getVelocity, (value) -> inputs.intakeWheelsSpeedRadPerSec = value);
        ifOk(
            intakeMotor, 
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values) -> inputs.intakeWheelsAppliedVoltage = values[0] * values[1]);
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeWheelsCurrantAmps = value);


        ifOk(wristMotor, wristEncoder::getPosition, (value) -> inputs.wristAngleRad = value);
        ifOk(wristMotor, wristEncoder::getVelocity, (value) -> inputs.wristSpeedRadsPerSec = value);
        ifOk(wristMotor, wristMotor::getOutputCurrent, (value) -> inputs.wristCurrantAmps = value);
        ifOk(
            wristMotor, 
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values) -> inputs.wristAppliedVoltage = values[0] * values[1]);

        inputs.proximitySensorDistance = proximitySensor.getValue();
    }

    @Override
    public void setAngle(double desiredangle) {
        double desiredPosition = desiredangle * wristMotorReduction / (2.0 * Math.PI);

        tryUntilOk(wristMotor, 5, () -> wristPID.setReference(desiredPosition, ControlType.kPosition));
    }

    @Override
    public void setSpeed(double desiredspeed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }
}
