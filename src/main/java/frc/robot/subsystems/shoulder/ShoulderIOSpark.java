package frc.robot.subsystems.shoulder;

import static frc.robot.subsystems.shoulder.ShoulderConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.BooleanSupplier;
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
        private final TrapezoidProfile.Constraints m_netConstraints = new TrapezoidProfile.Constraints(kMaxNetVelocity,
                        kMaxNetAcceleration);
        private final ConfigurableProfiledPIDController controller = new ConfigurableProfiledPIDController(kP, kI, kD,
                        m_constraints, "Shoulder Profiled PID Controller");
        private final ConfigurableProfiledPIDController netController = new ConfigurableProfiledPIDController(kP, kI,
                        kD,
                        m_netConstraints, "Shoulder Profiled PID Controller (net)");
        private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);
        private final ConfigurableParameter<Double> shoulderKs = new ConfigurableParameter<>(kS, "Shoulder kS",
                        feedforward::setKs);
        private final ConfigurableParameter<Double> shoulderKg = new ConfigurableParameter<>(kG, "Shoulder kG",
                        feedforward::setKg);
        private final ConfigurableParameter<Double> shoulderKv = new ConfigurableParameter<>(kV, "Shoulder kV",
                        feedforward::setKv);

        private final ArmFeedforward feedforwardNet = new ArmFeedforward(kS, kG, kV);
        private final ConfigurableParameter<Double> netKs = new ConfigurableParameter<>(kS, "Shoulder kS Net",
                        feedforwardNet::setKs);
        private final ConfigurableParameter<Double> netKg = new ConfigurableParameter<>(kG, "Shoulder kG Net",
                        feedforwardNet::setKg);
        private final ConfigurableParameter<Double> netKv = new ConfigurableParameter<>(kV, "Shoulder kV Net",
                        feedforwardNet::setKv);
        private final ConfigurableParameter<Double> safeZoneMin = new ConfigurableParameter<>(0.0,
                        "Shoulder Safe Zone Min");
        private final ConfigurableParameter<Double> safeZoneMax = new ConfigurableParameter<>(Math.PI,
                        "Shoulder Safe Zone Max");

        private boolean positionControlMode = false;
        private boolean netControlMode = false;
        private double currentTargetAngleRad;

        public ShoulderIOSpark() {
                controller.setIZone(1);
                controller.setIntegratorRange(-2, 2);
                tryUntilOk(leaderMotor, 5, () -> leaderMotor.configure(ShoulderConfig.leaderConfig,
                                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

                tryUntilOk(followerMotor, 5, () -> followerMotor.configure(ShoulderConfig.followerConfig,
                                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        }

        @Override
        public void updateInputs(ShoulderIOInputs inputs, boolean elevatorNotAtTarget) {
                double currentSetpoint = currentTargetAngleRad;
                if (elevatorNotAtTarget) {
                        currentSetpoint = Math.min(Math.max(currentSetpoint, safeZoneMin.get()), safeZoneMax.get());
                }
                if (positionControlMode) {
                        positionControlRio(currentSetpoint);
                }
                inputs.setpointAngleRad = currentSetpoint;
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
                inputs.inSafeZone = inputs.leaderShoulderAngleRad < safeZoneMax.get()
                                && inputs.leaderShoulderAngleRad > safeZoneMin.get();
        }

        @Override
        public void setShoulderAngle(double desiredAngle, boolean netAcceleration) {
                positionControlMode = true;
                currentTargetAngleRad = desiredAngle;
                netControlMode = netAcceleration;
        }

        public void setShoulderAngle(double desiredAngle) {
                setShoulderAngle(desiredAngle, false);
        }

        @Override
        public void resetPosControl() {
                controller.reset(encoder.getPosition() + kOffsetToHoriz, encoder.getVelocity());
                netController.reset(encoder.getPosition() + kOffsetToHoriz, encoder.getVelocity());
        }

        public void positionControlRio(double desiredAngle) {
                controller.disableContinuousInput();
                if (!netControlMode) {
                        double pidVal = controller.calculate(encoder.getPosition() + kOffsetToHoriz,
                                        desiredAngle + kOffsetToHoriz);
                        Logger.recordOutput("Shoulder Profile Pos",
                                        controller.getSetpoint().position);
                        Logger.recordOutput("Shoulder Profile vel",
                                        controller.getSetpoint().velocity);
                        leaderMotor.setVoltage(
                                        Math.min(pidVal, 7) + feedforward.calculate(controller.getSetpoint().position,
                                                        controller.getSetpoint().velocity));
                } else {
                        double pidVal = netController.calculate(encoder.getPosition() + kOffsetToHoriz,
                                        desiredAngle + kOffsetToHoriz);
                        Logger.recordOutput("Shoulder Profile Pos",
                                        netController.getSetpoint().position);
                        Logger.recordOutput("Shoulder Profile vel",
                                        netController.getSetpoint().velocity);
                        leaderMotor.setVoltage(
                                        Math.min(pidVal, 7)
                                                        + feedforwardNet.calculate(netController.getSetpoint().position,
                                                                        netController.getSetpoint().velocity));

                }
        }

        // TOD: check if we can use trapezoid profile itself
        // public void setShoulderAngleSpark(double desiredAngle) {
        // double lastSpeed = 0;
        // double lastTime = Timer.getFPGATimestamp();
        // controller.calculate(encoder.getPosition() + kOffsetToHoriz, desiredAngle);
        // double acceleration = (controller.getSetpoint().velocity - lastSpeed)
        // / (Timer.getFPGATimestamp() - lastTime);
        // leaderMotor.getClosedLoopController().setReference(controller.getSetpoint().position,
        // ControlType.kPosition, ClosedLoopSlot.kSlot0,
        // feedforward.calculate(controller.getSetpoint().velocity, acceleration));
        // }

        // TOD: test with and without FF
        // public void setShoulderAngleMaxMotion(double desiredAngle) {
        // double lastSpeed = 0;
        // double lastTime = Timer.getFPGATimestamp();
        // controller.calculate(encoder.getPosition() + kOffsetToHoriz, desiredAngle);
        // double acceleration = (controller.getSetpoint().velocity - lastSpeed)
        // / (Timer.getFPGATimestamp() - lastTime);
        // leaderMotor.getClosedLoopController().setReference(desiredAngle,
        // ControlType.kMAXMotionPositionControl,
        // ClosedLoopSlot.kSlot0,
        // feedforward.calculate(controller.getSetpoint().velocity, acceleration));
        // }

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
