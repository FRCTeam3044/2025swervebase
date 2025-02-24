package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSim implements ElevatorIO {
    private SparkMax sparkMax = new SparkMax(50, MotorType.kBrushless);
    private DCMotor gearBox = DCMotor.getNEO(2);
    private SparkMaxSim sparkMaxSim = new SparkMaxSim(sparkMax, gearBox);
    private RelativeEncoder elevatorEncoder = sparkMax.getEncoder();

    public final ElevatorSim m_elevatorSim = new ElevatorSim(gearBox,
            12.0,
            18.0,
            drumRadius,
            1,
            1.83,
            true,
            1,
            0.001,
            0.0);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(
            kP * 16,
            kI,
            kD,
            new TrapezoidProfile.Constraints(2.45, 2.45));
    ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    private double currentTargetMeters;
    private boolean positionControlMode = false;

    // public ElevatorIOSim() {
    // tryUntilOk(sparkMax, 5, () -> sparkMax.configure(ElevatorConfigs.leftConfig,
    // ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        m_elevatorSim.setInput(sparkMax.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_elevatorSim.update(0.020);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

        // Iterate the elevator and arm SPARK simulations
        sparkMaxSim.iterate(
                ((m_elevatorSim.getVelocityMetersPerSecond()
                        / (drumRadius * 2.0 * Math.PI * motorReduction))
                        * 60.0),
                RobotController.getBatteryVoltage(),
                0.02);

        if (positionControlMode) {
            m_controller.setGoal(currentTargetMeters);
            double pidOutput = m_controller.calculate(m_elevatorSim.getPositionMeters());
            double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
            sparkMax.setVoltage(pidOutput + feedforwardOutput);
        }
        // elevatorMotorSim.setAppliedOutput((pidOutput + feedforwardOutput) / 12.0);

        inputs.setpointMeters = currentTargetMeters;
        inputs.leaderPositionRot = elevatorEncoder.getPosition();
        inputs.elevatorHeightMeters = m_elevatorSim.getPositionMeters();
        // inputs.elevatorHeightCalc = (elevatorEncoder.getPosition() /
        // ElevatorConstants.kElevatorGearing)
        // * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI);
        inputs.leaderAppliedVolts = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.leaderCurrentAmps = sparkMaxSim.getMotorCurrent();
        // inputs.pidOutput = pidOutput;
        // inputs.feedforwardOutput = feedforwardOutput;
    }

    @Override
    public void setVoltage(double voltage) {
        positionControlMode = false;
        sparkMax.set(voltage);
    }

    @Override
    public void setSpeed(double desiredSpeed) {
        positionControlMode = false;
        sparkMax.set(desiredSpeed);
    }

    @Override
    public void setPosition(double position) {
        positionControlMode = true;
        currentTargetMeters = position;
    }
}
