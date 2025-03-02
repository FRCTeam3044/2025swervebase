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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkMax leaderMotor = new SparkMax(leaderCanId, MotorType.kBrushless);
    private final SparkMax followerMotor = new SparkMax(followerCanId, MotorType.kBrushless);

    private final RelativeEncoder encoder = leaderMotor.getAlternateEncoder();

    private final DigitalInput topHallEffect = new DigitalInput(0);
    private final DigitalInput bottomHallEffect = new DigitalInput(1);

    ConfigurableParameter<Double> bottomPoint = new ConfigurableParameter<Double>(0.0, "elevator/bottomPoint");
    ConfigurableParameter<Double> topPoint = new ConfigurableParameter<Double>(1.0, "elevator/topPoint");

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
            kMaxAcceleration);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV);

    private double currentTargetMeters;
    private boolean positionControlMode = false;

    private boolean shoulderInDangerZone = false;

    public ElevatorIOSpark() {
        tryUntilOk(followerMotor, 5, () -> followerMotor.configure(ElevatorConfigs.followerConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        tryUntilOk(leaderMotor, 5, () -> leaderMotor.configure(ElevatorConfigs.leaderConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void setPosition(double desiredPosition) {
        positionControlMode = true;
        currentTargetMeters = desiredPosition;
    }

    private void setPositionPPID(double desiredPosition) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        double pidVal = controller.calculate(encoder.getPosition(), desiredPosition);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        leaderMotor.setVoltage(
                pidVal
                        + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
        lastSpeed = controller.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }

    // TOD: check if we can use trapezoid profile itself
    public void setPositionSpark(double desiredPosition) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        controller.calculate(encoder.getPosition(), desiredPosition);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        leaderMotor.getClosedLoopController().setReference(controller.getSetpoint().position,
                ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    // TOD: test with and without FF
    public void setPositionMaxMotion(double desiredPosition) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        controller.calculate(encoder.getPosition(), desiredPosition);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        leaderMotor.getClosedLoopController().setReference(desiredPosition, ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0, feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    }

    @Override
    public void setSpeed(double desiredSpeed) {
        positionControlMode = false;

        if (!shoulderInDangerZone) {
            leaderMotor.set(desiredSpeed);
        }
    }

    @Override
    public void setVoltage(double voltage) {
        positionControlMode = false;

        if (!shoulderInDangerZone) {
            leaderMotor.setVoltage(voltage);
        }
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs, boolean shoulderInDangerZone) {
        this.shoulderInDangerZone = shoulderInDangerZone;
        double currentTargetRotations = currentTargetMeters / (drumRadius * 2.0 * Math.PI);

        if (shoulderInDangerZone) {
            leaderMotor.set(0);
        } else {
            if (positionControlMode)
                setPositionPPID(currentTargetRotations);
        }

        ifOk(leaderMotor, encoder::getPosition, (value) -> inputs.leaderPositionRot = value);
        ifOk(leaderMotor, encoder::getVelocity, (value) -> inputs.leaderVelocityRPM = value);
        ifOk(
                leaderMotor,
                new DoubleSupplier[] { leaderMotor::getAppliedOutput, leaderMotor::getBusVoltage },
                (values) -> inputs.leaderAppliedVolts = values[0] * values[1]);
        ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
        ifOk(leaderMotor, leaderMotor::getMotorTemperature, (value) -> inputs.leaderTemperature = value);
        ifOk(
                followerMotor,
                new DoubleSupplier[] { followerMotor::getAppliedOutput, followerMotor::getBusVoltage },
                (values) -> inputs.leaderAppliedVolts = values[0] * values[1]);
        ifOk(followerMotor, followerMotor::getMotorTemperature, (value) -> inputs.followerTemperature = value);
        inputs.setpointMeters = currentTargetMeters;
        inputs.setpointRotations = currentTargetRotations;
        inputs.elevatorHeightMeters = inputs.leaderPositionRot * drumRadius * 2.0 * Math.PI;
        inputs.bottomEffectClosed = bottomHallEffect.get();
        inputs.topHallEffectClosed = topHallEffect.get();

        if (bottomHallEffect.get()) {
            encoder.setPosition(bottomPoint.get());
        }

        // if (topHallEffect.get()) {
        // encoder.setPosition(topPoint.get());
        // }
    }

    @Override
    public void resetPosControl() {
        controller.reset(encoder.getPosition(), encoder.getVelocity());
    }
}
