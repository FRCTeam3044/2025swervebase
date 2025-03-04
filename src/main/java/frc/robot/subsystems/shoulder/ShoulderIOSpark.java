package frc.robot.subsystems.shoulder;

import static frc.robot.subsystems.shoulder.ShoulderConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;

@SuppressWarnings("unused")
public class ShoulderIOSpark implements ShoulderIO {
        private final SparkMax leaderMotor = new SparkMax(leaderCanId, MotorType.kBrushless);
        private final SparkMax followerMotor = new SparkMax(followerCanId, MotorType.kBrushless);
        private final AbsoluteEncoder encoder = leaderMotor.getAbsoluteEncoder();

        private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
                        kMaxAcceleration);
        private final ConfigurableProfiledPIDController controller = new ConfigurableProfiledPIDController(kP, kI, kD,
                        m_constraints, "Shoulder Profiled PID Controller");
        private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);
        private final ConfigurableParameter<Double> shoulderKs = new ConfigurableParameter<>(kS, "Shoulder kS",
                        feedforward::setKs);
        private final ConfigurableParameter<Double> shoulderKg = new ConfigurableParameter<>(kG, "Shoulder kG",
                        feedforward::setKg);
        private final ConfigurableParameter<Double> shoulderKv = new ConfigurableParameter<>(kV, "Shoulder kV",
                        feedforward::setKv);

        private boolean positionControlMode = false;
        private double currentTargetAngleRad;

        public ShoulderIOSpark() {
                tryUntilOk(leaderMotor, 5, () -> leaderMotor.configure(ShoulderConfig.leaderConfig,
                                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

                tryUntilOk(followerMotor, 5, () -> followerMotor.configure(ShoulderConfig.followerConfig,
                                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        }

        @Override
        public void updateInputs(ShoulderIOInputs inputs) {
                if (positionControlMode) {
                        positionControlRio(currentTargetAngleRad);
                }
                inputs.setpointAngleRad = currentTargetAngleRad;
                inputs.setpointAngleFromHorizontal = currentTargetAngleRad + kOffsetToHoriz;
                ifOk(leaderMotor, encoder::getPosition, (value) -> inputs.leaderShoulderAngleRad = value);
                ifOk(leaderMotor, encoder::getPosition,
                                (value) -> inputs.leaderShoulderAngleFromHorizontal = value + kOffsetToHoriz);
                ifOk(leaderMotor, encoder::getVelocity,
                                (value) -> inputs.leaderShoulderSpeedRadsPerSec = value);
                ifOk(leaderMotor, leaderMotor::getOutputCurrent,
                                (value) -> inputs.leaderShoulderCurrentAmps = value);
                ifOk(
                                leaderMotor,
                                new DoubleSupplier[] { leaderMotor::getAppliedOutput,
                                                leaderMotor::getBusVoltage },
                                (values) -> inputs.leaderShoulderAppliedVoltage = values[0] * values[1]);
                ifOk(leaderMotor, leaderMotor::getMotorTemperature,
                                (value) -> inputs.leaderTemperature = value);
                ifOk(followerMotor, followerMotor::getMotorTemperature,
                                (value) -> inputs.followerTemperature = value);
        }

        @Override
        public void setShoulderAngle(double desiredAngle) {
                positionControlMode = true;
                currentTargetAngleRad = desiredAngle;
        }

        @Override
        public void resetPosControl() {
                controller.reset(encoder.getPosition() + kOffsetToHoriz, encoder.getVelocity());
        }

        public void positionControlRio(double desiredAngle) {
                // double lastSpeed = 0;
                // double lastTime = Timer.getFPGATimestamp();
                controller.disableContinuousInput();
                double pidVal = controller.calculate(encoder.getPosition() + kOffsetToHoriz,
                                desiredAngle + kOffsetToHoriz);
                // Logger.recordOutput("Shoulder Profile Pos",
                // controller.getSetpoint().position);
                // Logger.recordOutput("Shoulder Profile vel",
                // controller.getSetpoint().velocity);

                // double acceleration = (controller.getSetpoint().velocity - lastSpeed)
                // / (Timer.getFPGATimestamp() - lastTime);
                // leaderMotor.setVoltage(feedforward.calculate(desiredAngle + kOffsetToHoriz,
                // 0));
                leaderMotor.setVoltage(Math.min(pidVal, 7) + feedforward.calculate(controller.getSetpoint().position,
                                controller.getSetpoint().velocity));
                // lastSpeed = controller.getSetpoint().velocity;
                // lastTime = Timer.getFPGATimestamp();
        }

        // TODO: check if we can use trapezoid profile itself
        public void setShoulderAngleSpark(double desiredAngle) {
                double lastSpeed = 0;
                double lastTime = Timer.getFPGATimestamp();
                controller.calculate(encoder.getPosition() + kOffsetToHoriz, desiredAngle);
                double acceleration = (controller.getSetpoint().velocity - lastSpeed)
                                / (Timer.getFPGATimestamp() - lastTime);
                leaderMotor.getClosedLoopController().setReference(controller.getSetpoint().position,
                                ControlType.kPosition, ClosedLoopSlot.kSlot0,
                                feedforward.calculate(controller.getSetpoint().velocity, acceleration));
        }

        // TODO: test with and without FF
        public void setShoulderAngleMaxMotion(double desiredAngle) {
                double lastSpeed = 0;
                double lastTime = Timer.getFPGATimestamp();
                controller.calculate(encoder.getPosition() + kOffsetToHoriz, desiredAngle);
                double acceleration = (controller.getSetpoint().velocity - lastSpeed)
                                / (Timer.getFPGATimestamp() - lastTime);
                leaderMotor.getClosedLoopController().setReference(desiredAngle,
                                ControlType.kMAXMotionPositionControl,
                                ClosedLoopSlot.kSlot0,
                                feedforward.calculate(controller.getSetpoint().velocity, acceleration));
        }

        @Override
        public void setShoulderSpeed(double desiredSpeed) {
                positionControlMode = false;
                leaderMotor.set(desiredSpeed);
        }

        @Override
        public void setVoltage(double voltage) {
                positionControlMode = false;
                leaderMotor.setVoltage(voltage);
        }
}
