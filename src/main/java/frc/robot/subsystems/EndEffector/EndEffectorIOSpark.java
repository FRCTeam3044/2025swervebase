package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import me.nabdev.oxconfig.sampleClasses.ConfigurableSparkClosedLoop;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax intakeMotor = new SparkMax(rollerCanId, MotorType.kBrushless);
    private final SparkMax wristMotor = new SparkMax(wristCanId, MotorType.kBrushless);
    private final AbsoluteEncoder wristEncoder = intakeMotor.getAbsoluteEncoder();
    private final AnalogInput proximitySensor = new AnalogInput(proximitySensorChannel);

    private final SparkClosedLoopController wristPID = wristMotor.getClosedLoopController();

    public EndEffectorIOSpark() {
        var intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        intakeConfig.encoder
                .positionConversionFactor(2.0 * Math.PI / intakeMotorReduction)
                .velocityConversionFactor((2.0 * Math.PI) / 60.0 / intakeMotorReduction)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        tryUntilOk(intakeMotor, 5, () -> intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

        var wristConfig = new SparkMaxConfig();
        wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        wristConfig.encoder
                .positionConversionFactor(2.0 * Math.PI / wristMotorReduction)
                .velocityConversionFactor((2.0 * Math.PI) / 60.0 / wristMotorReduction)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        new ConfigurableSparkClosedLoop(wristConfig, wristMotor.configAccessor, wristMotor, "wrist");

        tryUntilOk(wristMotor, 5, () -> wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        ifOk(intakeMotor, wristEncoder::getVelocity, (value) -> inputs.intakeWheelsSpeedRadPerSec = value);
        ifOk(
                intakeMotor,
                new DoubleSupplier[] { intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage },
                (values) -> inputs.intakeWheelsAppliedVoltage = values[0] * values[1]);
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeWheelsCurrantAmps = value);

        ifOk(wristMotor, wristEncoder::getPosition, (value) -> inputs.wristAngleRad = value);
        ifOk(wristMotor, wristEncoder::getVelocity, (value) -> inputs.wristSpeedRadsPerSec = value);
        ifOk(wristMotor, wristMotor::getOutputCurrent, (value) -> inputs.wristCurrantAmps = value);
        ifOk(
                wristMotor,
                new DoubleSupplier[] { intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage },
                (values) -> inputs.wristAppliedVoltage = values[0] * values[1]);

        inputs.proximitySensorDistance = proximitySensor.getValue();
    }

    @Override
    public void setWristAngle(double angle) {
        wristPID.setReference(angle, ControlType.kPosition);
    }

    @Override
    public void setWristSpeed(double desiredspeed) {
        wristMotor.set(desiredspeed);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

}
