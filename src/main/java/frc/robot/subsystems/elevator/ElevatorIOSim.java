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
            0.6985,
            2.0,
            true,
            0,
            0.01,
            0.0);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(2.45, 2.45));
    ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    private double currentTarget;
    private boolean positionControlMode = false;

    // public ElevatorIOSim() {
    //     tryUntilOk(sparkMax, 5, () -> sparkMax.configure(ElevatorConfigs.leftConfig,
    //             ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    //}

    public void simulationPeriodic() {
        m_elevatorSim.setInput(sparkMaxSim.getVelocity() * RobotController.getBatteryVoltage());
        m_elevatorSim.update(0.020);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        m_elevatorSim.setInput(sparkMax.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_elevatorSim.update(0.020);

        // Iterate the elevator and arm SPARK simulations
        sparkMaxSim.iterate(
                ((m_elevatorSim.getVelocityMetersPerSecond()
                        / (drumRadius * 2.0 * Math.PI))
                        * motorReduction)
                        * 60.0,
                RobotController.getBatteryVoltage(),
                0.02);

        if (positionControlMode) {
            m_controller.setGoal(currentTarget);
            // With the setpoint value we run PID control like normal
            double pidOutput = m_controller.calculate(elevatorEncoder.getPosition());
            double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
            sparkMax.set((pidOutput + feedforwardOutput) / 12.0);
        }
        // elevatorMotorSim.setAppliedOutput((pidOutput + feedforwardOutput) / 12.0);

        inputs.setpoint = currentTarget;
        inputs.leftPositionRot = elevatorEncoder.getPosition();
        inputs.elevatorHeight = m_elevatorSim.getPositionMeters();
        // inputs.elevatorHeightCalc = (elevatorEncoder.getPosition() /
        // ElevatorConstants.kElevatorGearing)
        // * (ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI);
        inputs.leftAppliedVolts = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.leftCurrentAmps = sparkMaxSim.getMotorCurrent();
        // inputs.pidOutput = pidOutput;
        // inputs.feedforwardOutput = feedforwardOutput;
    }

    @Override
    public void setVoltage(double voltage) {
        sparkMax.set(voltage);
    }

    @Override
    public void setSpeed(double desiredSpeed) {
        sparkMax.set(desiredSpeed);
    }

    @Override
    public void setPosition(double position) {
        currentTarget = position;
    }
}
