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
    private final SparkMax climberOne = new SparkMax(climberCanId, MotorType.kBrushed);
    private final SparkMax climberTwo = new SparkMax(climberCanId, MotorType.kBrushed);
    private final RelativeEncoder climberEncoderOne = climberOne.getEncoder();
    private final RelativeEncoder climberEncoderTwo = climberTwo.getEncoder();


    public ClimberIOSpark(){
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        config
        .encoder
        .positionConversionFactor(
            2.0 * Math.PI / motorReduction) // Motor Rotations -> Roller Radians
        .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    tryUntilOk(
        climberOne,
        5,
        () ->
            climberOne.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(climberOne, climberEncoderOne::getPosition, (value) -> inputs.positionRad = value);
    ifOk(climberOne, climberEncoderOne::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        climberOne,
        new DoubleSupplier[] {climberOne::getAppliedOutput, climberOne::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(climberOne, climberOne::getOutputCurrent, (value) -> inputs.currentAmps = value);

  }


  

  @Override
  public void setSpeed(double speed) {
    // TODO: Auto-generated method stub
  }

}