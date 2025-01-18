package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static frc.robot.util.SparkUtil.*;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimberIOSpark implements ClimberIO {
    private final SparkMax climber = new SparkMax(climberCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder = climber.getEncoder();


    public ClimberIOSpark(){
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        climber,
        5,
        () ->
            climber.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(climber, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(climber, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        climber,
        new DoubleSupplier[] {climber::getAppliedOutput, climber::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(climber, climber::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setPositionRadians(double radians) {
    // TODO: Auto-generated method stub
  }

}