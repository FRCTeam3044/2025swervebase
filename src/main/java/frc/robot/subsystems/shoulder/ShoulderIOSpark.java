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
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class ShoulderIOSpark implements ShoulderIO {
    private final SparkMax shoulderMotorLeft = new SparkMax(shoulderOneCanId, MotorType.kBrushless);
    private final SparkMax shoulderMotorRight = new SparkMax(shoulderTwoCanId, MotorType.kBrushless);
    private final AbsoluteEncoder shoulderEncoder = shoulderMotorLeft.getAbsoluteEncoder();

    public static SoftLimitConfig leftSoftConfig = new SoftLimitConfig();
    public static SoftLimitConfig rightSoftConfig = new SoftLimitConfig();

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
            kMaxAcceleration);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

    public ShoulderIOSpark() {
        var shoulderMotorLeftConfig = new SparkMaxConfig();

        leftSoftConfig.forwardSoftLimit(0.0);
        leftSoftConfig.reverseSoftLimit(0.0);

        shoulderMotorLeftConfig.apply(leftSoftConfig);

        shoulderMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        shoulderMotorLeftConfig.encoder
                .positionConversionFactor(2.0 * Math.PI / shoulderMotorReduction)
                .velocityConversionFactor((2.0 * Math.PI) / 60.0 / shoulderMotorReduction)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        tryUntilOk(shoulderMotorLeft, 5, () -> shoulderMotorLeft.configure(shoulderMotorLeftConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        var shoulderMotorRightConfig = new SparkMaxConfig();

        rightSoftConfig.forwardSoftLimit(0.0);
        rightSoftConfig.reverseSoftLimit(0.0);

        shoulderMotorRightConfig.apply(rightSoftConfig);

        shoulderMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        shoulderMotorRightConfig.encoder
                .positionConversionFactor(2.0 * Math.PI / shoulderMotorReduction)
                .velocityConversionFactor((2.0 * Math.PI) / 60.0 / shoulderMotorReduction)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        shoulderMotorRightConfig.follow(shoulderMotorLeft);
        tryUntilOk(shoulderMotorRight, 5, () -> shoulderMotorRight.configure(shoulderMotorLeftConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        ifOk(shoulderMotorLeft, shoulderEncoder::getPosition, (value) -> inputs.leftShoulderAngleRad = value);
        ifOk(shoulderMotorLeft, shoulderEncoder::getVelocity, (value) -> inputs.leftShoulderSpeedRadsPerSec = value);
        ifOk(shoulderMotorLeft, shoulderMotorLeft::getOutputCurrent, (value) -> inputs.leftShoulderCurrentAmps = value);
        ifOk(
                shoulderMotorLeft,
                new DoubleSupplier[] { shoulderMotorLeft::getAppliedOutput, shoulderMotorLeft::getBusVoltage },
                (values) -> inputs.leftShoulderAppliedVoltage = values[0] * values[1]);
        ifOk(shoulderMotorLeft, shoulderMotorLeft::getMotorTemperature, (value) -> inputs.leftTemperature = value );
        ifOk(shoulderMotorRight, shoulderEncoder::getPosition, (value) -> inputs.rightShoulderAngleRad = value);
        ifOk(shoulderMotorRight, shoulderEncoder::getVelocity, (value) -> inputs.rightShoulderSpeedRadsPerSec = value);
        ifOk(shoulderMotorRight, shoulderMotorRight::getOutputCurrent, (value) -> inputs.rightShoulderCurrentAmps = value);
        ifOk(
                shoulderMotorRight,
                new DoubleSupplier[] { shoulderMotorRight::getAppliedOutput, shoulderMotorRight::getBusVoltage },
                (values) -> inputs.rightShoulderAppliedVoltage = values[0] * values[1]);
        ifOk(shoulderMotorRight, shoulderMotorRight::getMotorTemperature, (value) -> inputs.rightTemperature = value );
    }

    @Override
    public void setShoulderAngle(double desiredAngle) {
        // TODO Auto-generated method stub
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        double pidVal = controller.calculate(shoulderEncoder.getPosition(), desiredAngle);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        shoulderMotorLeft.setVoltage(
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
        shoulderMotorLeft.getClosedLoopController().setReference(controller.getSetpoint().position,
                ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    // TODO: test with and without FF
    public void setShoulderAngleMaxMotion(double desiredAngle) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        controller.calculate(shoulderEncoder.getPosition(), desiredAngle);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        shoulderMotorLeft.getClosedLoopController().setReference(desiredAngle, ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0, feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    @Override
    public void setShoulderSpeed(double desiredSpeed) {
        // TODO Auto-generated method stub
        shoulderMotorLeft.set(desiredSpeed);
    }

    @Override
    public void setVoltage(double voltage) {
        shoulderMotorLeft.setVoltage(voltage);
    }
}
