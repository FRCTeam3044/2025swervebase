package frc.robot.subsystems.shoulder;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.shoulder.ShoulderConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShoulderIOSim implements ShoulderIO {
    private SparkMax sparkMax = new SparkMax(52, MotorType.kBrushless);

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
            kMaxAcceleration);
    private final ProfiledPIDController controller = new ProfiledPIDController(12.527, kI, 0.723, m_constraints, kDt);

    private DCMotor gearBox = DCMotor.getNEO(2);
    private SparkMaxSim sparkMaxSim = new SparkMaxSim(sparkMax, gearBox);
    SingleJointedArmSim shoulderSim = new SingleJointedArmSim(gearBox, 35, 1.0,
            Inches.of(15.75).in(Meters), -Math.PI / 2,
            1.5 * Math.PI, true, 0.0, 0, 0);

    ArmFeedforward feedforward = new ArmFeedforward(0.073344, 1.3321, 0.86874);

    private boolean positionControlMode = false;
    private double currentTargetAngleRad;

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        Logger.recordOutput("ShoulderPosControl", positionControlMode);
        Logger.recordOutput("ShoulderTarget", currentTargetAngleRad);
        if (positionControlMode) {
            controller.setGoal(currentTargetAngleRad);
            double pidOutput = controller.calculate(shoulderSim.getAngleRads());
            double targetVel = controller.getSetpoint().velocity;
            double feedforwardOutput = feedforward.calculate(controller.getSetpoint().position, targetVel);
            sparkMax.setVoltage(pidOutput + feedforwardOutput);
            Logger.recordOutput("TargetPosition", controller.getSetpoint().position);
            Logger.recordOutput("TargetVelocity", targetVel);
            Logger.recordOutput("PID Output", pidOutput);
            Logger.recordOutput("Feedforward Output", feedforwardOutput);
            Logger.recordOutput("ShoulderAppliedVoltage", pidOutput + feedforwardOutput);
        }
        shoulderSim.setInput(sparkMax.getAppliedOutput() * RobotController.getBatteryVoltage());
        shoulderSim.update(0.020);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(shoulderSim.getCurrentDrawAmps()));
        sparkMaxSim.iterate(
                (shoulderSim.getVelocityRadPerSec() * 60 * shoulderMotorReduction / (2 * Math.PI)),
                RobotController.getBatteryVoltage(),
                0.02);
        inputs.leaderShoulderAngleRad = shoulderSim.getAngleRads();
        inputs.leaderShoulderSpeedRadsPerSec = shoulderSim.getVelocityRadPerSec();
        inputs.leaderShoulderRots = sparkMax.getEncoder().getPosition();
        inputs.leaderShoulderSpeedRPM = sparkMax.getEncoder().getVelocity();
        inputs.leaderShoulderAppliedVoltage = sparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.setpointAngleRad = currentTargetAngleRad;
    }

    @Override
    public void setShoulderAngle(double desiredAngle) {
        positionControlMode = true;
        currentTargetAngleRad = desiredAngle;
    }

    @Override
    public void setVoltage(double voltage) {
        positionControlMode = false;
        sparkMax.setVoltage(voltage);
    }

    @Override
    public void setShoulderSpeed(double desiredSpeed) {
        positionControlMode = false;
        sparkMax.set(desiredSpeed);
    }
}
