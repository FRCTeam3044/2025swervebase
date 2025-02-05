package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.util.SparkUtil.*;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimberIOSpark implements ClimberIO {
    private final SparkMax rightMotorClimber = new SparkMax(rightMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder1 = rightMotorClimber.getEncoder();

    private final SparkMax leftMotorClimber = new SparkMax(leftMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder2 = leftMotorClimber.getEncoder();


  /*   public ClimberIOSpark(){
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
  } */

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(rightMotorClimber, encoder1::getPosition, (value) -> inputs.positionRad = value);
    ifOk(rightMotorClimber, encoder1::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
      rightMotorClimber,
        new DoubleSupplier[] {rightMotorClimber::getAppliedOutput, rightMotorClimber::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(rightMotorClimber, rightMotorClimber::getOutputCurrent, (value) -> inputs.currentAmps = value);

    
    ifOk(leftMotorClimber, encoder2::getPosition, (value) -> inputs.positionRad = value);
    ifOk(leftMotorClimber, encoder2::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
      leftMotorClimber,
        new DoubleSupplier[] {leftMotorClimber::getAppliedOutput, leftMotorClimber::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(leftMotorClimber, leftMotorClimber::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }


  

  @Override
  public void setSpeed(double speed) {
    leftMotorClimber.set(speed);
  }
}