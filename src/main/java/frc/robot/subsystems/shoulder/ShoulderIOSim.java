package frc.robot.subsystems.shoulder;

import static frc.robot.subsystems.shoulder.ShoulderConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
    private final SparkMax sparkMax = new SparkMax(52, MotorType.kBrushless);
    private final AbsoluteEncoder shoulderEncoder = sparkMax.getAbsoluteEncoder();

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
            kMaxAcceleration);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
    ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV);

    private DCMotor gearBox = DCMotor.getNEO(2);
    private SparkMaxSim sparkMaxSim = new SparkMaxSim(sparkMax, gearBox);
    SingleJointedArmSim shoulderSim = new SingleJointedArmSim(gearBox, 48.0, 1.0, 1.0, 0.0, 0.0, false, 0.0, 0, 0);

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        shoulderSim.setInput(sparkMax.getAppliedOutput() * RobotController.getBatteryVoltage());
        shoulderSim.update(0.020);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(shoulderSim.getCurrentDrawAmps()));
        sparkMaxSim.iterate(
                (shoulderSim.getVelocityRadPerSec()),
                RobotController.getBatteryVoltage(),
                0.02);
        inputs.leftShoulderAngleRad = shoulderSim.getAngleRads();
    }

    @Override
    public void setShoulderAngle(double desiredAngle) {
        double lastSpeed = 0;
        double lastTime = Timer.getFPGATimestamp();
        double pidVal = controller.calculate(shoulderEncoder.getPosition(), desiredAngle);
        double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
        sparkMax.setVoltage(
                pidVal
                        + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
        lastSpeed = controller.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setVoltage(double voltage) {
        sparkMax.setVoltage(voltage);
    }

    @Override
    public void setShoulderSpeed(double desiredSpeed) {
        sparkMax.set(desiredSpeed);
    }
}
