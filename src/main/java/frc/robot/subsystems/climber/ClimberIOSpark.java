package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.util.SparkUtil.*;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimberIOSpark implements ClimberIO {
  private final SparkMax rightMotorClimber = new SparkMax(rightMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder1 = rightMotorClimber.getEncoder();

  private final SparkMax leftMotorClimber = new SparkMax(leftMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder2 = leftMotorClimber.getEncoder();

  public ClimberIOSpark() {
    tryUntilOk(rightMotorClimber, 5, () -> rightMotorClimber.configure(ClimberConfig.rightMotorClimber,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(leftMotorClimber, 5, () -> leftMotorClimber.configure(ClimberConfig.leftMotorClimber,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(rightMotorClimber, encoder1::getPosition, (value) -> inputs.positionRad = value);
    ifOk(rightMotorClimber, encoder1::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        rightMotorClimber,
        new DoubleSupplier[] { rightMotorClimber::getAppliedOutput, rightMotorClimber::getBusVoltage },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(rightMotorClimber, rightMotorClimber::getOutputCurrent, (value) -> inputs.currentAmps = value);

    ifOk(leftMotorClimber, encoder2::getPosition, (value) -> inputs.positionRad = value);
    ifOk(leftMotorClimber, encoder2::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        leftMotorClimber,
        new DoubleSupplier[] { leftMotorClimber::getAppliedOutput, leftMotorClimber::getBusVoltage },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(leftMotorClimber, leftMotorClimber::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setSpeed(double speed) {
    // TODO: Auto-generated method stub
    rightMotorClimber.set(speed);
    leftMotorClimber.set(speed);
  }
}