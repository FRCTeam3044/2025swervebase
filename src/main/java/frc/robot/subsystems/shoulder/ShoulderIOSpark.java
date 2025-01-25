package frc.robot.subsystems.shoulder;

import static frc.robot.subsystems.shoulder.ShoulderConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class ShoulderIOSpark implements ShoulderIO {
    private final SparkMax shoulderMotor = new SparkMax(shoulderCanId, MotorType.kBrushless);
    private final AbsoluteEncoder shoulderEncoder = shoulderMotor.getAbsoluteEncoder();

      private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
      private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);
    
    public ShoulderIOSpark() {
        var shoulderMotorConfig = new SparkMaxConfig();
        shoulderMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        shoulderMotorConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / shoulderMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / shoulderMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(shoulderMotor, 5, () -> shoulderMotor.configure(shoulderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        ifOk(shoulderMotor, shoulderEncoder::getPosition, (value) -> inputs.shoulderAngleRad = value);
        ifOk(shoulderMotor, shoulderEncoder::getVelocity, (value) -> inputs.shoulderSpeedRadsPerSec = value);
        ifOk(shoulderMotor, shoulderMotor::getOutputCurrent, (value) -> inputs.shoulderCurrantAmps = value);
        ifOk(
                shoulderMotor,
                new DoubleSupplier[] { shoulderMotor::getAppliedOutput, shoulderMotor::getBusVoltage },
                (values) -> inputs.shoulderAppliedVoltage = values[0] * values[1]);
    }

    @Override
    public void setShoulderAngle(double desiredAngle) {
        // TODO Auto-generated method stub
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        double pidVal = controller.calculate(shoulderEncoder.getPosition(), desiredAngle);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        shoulderMotor.setVoltage(
            pidVal
            + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
        lastSpeed = controller.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();   
    }

    // TODO: check if we can use trapezoid profile itself
    public void setShoulderAngleSpark(double desiredAngle) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        controller.calculate(shoulderEncoder.getPosition(), desiredAngle);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        shoulderMotor.getClosedLoopController().setReference(controller.getSetpoint().position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    // TODO: test with and without FF
    public void setShoulderAngleMaxMotion(double desiredAngle) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        controller.calculate(shoulderEncoder.getPosition(), desiredAngle);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        shoulderMotor.getClosedLoopController().setReference(desiredAngle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    @Override
    public void setShoulderSpeed(double desiredSpeed) {
        // TODO Auto-generated method stub
        shoulderMotor.set(desiredSpeed);
    }
}
