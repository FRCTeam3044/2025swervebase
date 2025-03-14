package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import me.nabdev.oxconfig.ConfigurableParameter;

import static frc.robot.util.SparkUtil.*;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimberIOSpark implements ClimberIO {
  private final SparkMax leaderMotor = new SparkMax(leaderCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();

  private final SparkMax followerMotor = new SparkMax(followerCanId, MotorType.kBrushless);

  private final Servo servo = new Servo(servoPwmPort);
  private boolean servoClosed = true;

  private final ConfigurableParameter<Double> servoOpenSpeed = new ConfigurableParameter<>(1.0,
      "Servo Open Speed");

  public ClimberIOSpark() {
    tryUntilOk(leaderMotor, 5, () -> leaderMotor.configure(ClimberConfig.leaderConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(followerMotor, 5, () -> followerMotor.configure(ClimberConfig.followerConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    ifOk(leaderMotor, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(leaderMotor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        leaderMotor,
        new DoubleSupplier[] { leaderMotor::getAppliedOutput, leaderMotor::getBusVoltage },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.servoPosition = servo.getAngle();

    if (leaderMotor.getReverseLimitSwitch().isPressed()) {
      leaderMotor.getEncoder().setPosition(0);
    }

    servo.setSpeed(servoClosed ? -servoOpenSpeed.get() : servoOpenSpeed.get());
  }

  @Override
  public void setSpeed(double speed) {
    leaderMotor.set(speed);
  }

  @Override
  public void setServoClosed(boolean closed) {
    servoClosed = closed;
  }
}