package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import me.nabdev.oxconfig.ConfigurableParameter;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkMax leftElevatorMotor = new SparkMax(leftCanId, MotorType.kBrushless);
    private final SparkMax rightElevatorMotor = new SparkMax(rightCanId, MotorType.kBrushless);

    private final RelativeEncoder elevatorEncoder = leftElevatorMotor.getEncoder();

    private final DigitalInput topHallEffect = new DigitalInput(0);
    private final DigitalInput bottomHallEffect = new DigitalInput(1);

    ConfigurableParameter<Double> bottomPoint = new ConfigurableParameter<Double>(0.0, "elevator/bottomPoint");
    ConfigurableParameter<Double> topPoint = new ConfigurableParameter<Double>(1.0, "elevator/topPoint");

      private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
      private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
      private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV);

    public ElevatorIOSpark() {

        SparkMaxConfig configRight = new SparkMaxConfig();
        configRight.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).encoder
                .positionConversionFactor(0);

        tryUntilOk(rightElevatorMotor, 5, () -> rightElevatorMotor.configure(configRight,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        SparkMaxConfig configLeft = new SparkMaxConfig();
        configLeft.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);

        tryUntilOk(leftElevatorMotor, 5, () -> leftElevatorMotor.configure(configLeft, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

        configRight.follow(leftElevatorMotor);
    }

    @Override
    public void setPosition(double desiredPosition) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        double pidVal = controller.calculate(elevatorEncoder.getPosition(), desiredPosition);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        leftElevatorMotor.setVoltage(
            pidVal
            + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
        lastSpeed = controller.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();       
    }

    // TODO: check if we can use trapezoid profile itself
    public void setPositionSpark(double desiredPosition) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        controller.calculate(elevatorEncoder.getPosition(), desiredPosition);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        leftElevatorMotor.getClosedLoopController().setReference(controller.getSetpoint().position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    // TODO: test with and without FF
    public void setPositionMaxMotion(double desiredPosition) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        controller.calculate(elevatorEncoder.getPosition(), desiredPosition);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        leftElevatorMotor.getClosedLoopController().setReference(desiredPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    @Override
    public void setSpeed(double desiredSpeed) {
        leftElevatorMotor.set(desiredSpeed);
    }

    @Override
    public void setVoltage(double voltage) {
        leftElevatorMotor.setVoltage(voltage);
    } 

    public void updateInputs(ElevatorIOInputs inputs) {
        ifOk(leftElevatorMotor, elevatorEncoder::getPosition, (value) -> inputs.leftPositionRad = value);
        ifOk(leftElevatorMotor, elevatorEncoder::getVelocity, (value) -> inputs.leftVelocityRadPerSec = value);
        ifOk(
                leftElevatorMotor,
                new DoubleSupplier[] { leftElevatorMotor::getAppliedOutput, leftElevatorMotor::getBusVoltage },
                (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
        ifOk(leftElevatorMotor, leftElevatorMotor::getOutputCurrent, (value) -> inputs.leftCurrentAmps = value);
        ifOk(leftElevatorMotor, leftElevatorMotor::getMotorTemperature, (value) -> inputs.leftTemperature = value );
        ifOk(rightElevatorMotor, elevatorEncoder::getPosition, (value) -> inputs.rightPositionRad = value);
        ifOk(rightElevatorMotor, elevatorEncoder::getVelocity, (value) -> inputs.rightVelocityRadPerSec = value);
        ifOk(
                rightElevatorMotor,
                new DoubleSupplier[] { rightElevatorMotor::getAppliedOutput, rightElevatorMotor::getBusVoltage },
                (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
        ifOk(rightElevatorMotor, rightElevatorMotor::getOutputCurrent, (value) -> inputs.rightCurrentAmps = value);
        ifOk(rightElevatorMotor, rightElevatorMotor::getMotorTemperature, (value) -> inputs.rightTemperature = value );
        inputs.bottomEffectClosed = bottomHallEffect.get();
        inputs.topHallEffectClosed = topHallEffect.get();

        if(bottomHallEffect.get()) {
            elevatorEncoder.setPosition(bottomPoint.get());
        }

        if(topHallEffect.get()) {
            elevatorEncoder.setPosition(topPoint.get());
        }
    }
}
