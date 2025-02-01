package frc.robot.subsystems.shoulder;

import static frc.robot.util.SparkUtil.tryUntilOk;

import static frc.robot.subsystems.shoulder.ShoulderConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShoulderIOSim implements ShoulderIO {
    private final SparkMax shoulderMotorLeft = new SparkMax(52, MotorType.kBrushless);
    private final SparkMax shoulderMotorRight = new SparkMax(53, MotorType.kBrushless);
    private final AbsoluteEncoder shoulderEncoder = shoulderMotorLeft.getAbsoluteEncoder();

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

    private DCMotor gearBox = DCMotor.getNEO(2);
    private SparkMaxSim sparkMaxSim = new SparkMaxSim(shoulderMotorLeft, gearBox);
    SingleJointedArmSim shoulderSim = new SingleJointedArmSim(gearBox, 48.0, 0.0, 1.0, 0.0, 0.0, false, 0.0, null);

    // Add to Robot.java
    public void simulationPeriodic() {
        shoulderSim.setInput(sparkMaxSim.getVelocity() * RobotController.getBatteryVoltage());
        shoulderSim.update(0.020);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(shoulderSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        var shoulderMotorLeftConfig = new SparkMaxConfig();
        shoulderMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        shoulderMotorLeftConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / shoulderMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / shoulderMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        tryUntilOk(shoulderMotorLeft, 5, () -> shoulderMotorLeft.configure(shoulderMotorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        var shoulderMotorRightConfig = new SparkMaxConfig();
        shoulderMotorRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        shoulderMotorRightConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / shoulderMotorReduction)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / shoulderMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        shoulderMotorRightConfig.follow(shoulderMotorLeft);
        tryUntilOk(shoulderMotorRight, 5, () -> shoulderMotorRight.configure(shoulderMotorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    @Override
    public void setShoulderAngle(double desiredAngle) {
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

    @Override
    public void setVoltage(double voltage) {
        shoulderMotorLeft.setVoltage(voltage);
    }

    @Override
    public void setShoulderSpeed(double desiredSpeed) {
        shoulderMotorLeft.set(desiredSpeed);
    }
}
