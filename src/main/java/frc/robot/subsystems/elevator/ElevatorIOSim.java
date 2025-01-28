package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSim implements ElevatorIO{
    private SparkMax sparkMax = new SparkMax(50, MotorType.kBrushless);
    private DCMotor gearBox = DCMotor.getNEO(2);
    private SparkMaxSim sparkMaxSim = new SparkMaxSim(sparkMax, gearBox);

    public final ElevatorSim m_elevatorSim =
    new ElevatorSim(gearBox, 
    12.0,
    18.0,
    0.000000000000000000000000000000000000000000000000000000001,
    0.6985,
    2.0,
    true,
    0,
    0.01,
    0.0);

    public void simulationPeriodic() {
        m_elevatorSim.setInput(sparkMaxSim.getVelocity() * RobotController.getBatteryVoltage());
        m_elevatorSim.update(0.020);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        
    }

    @Override
    public void setVoltage(double voltage) {

    }

    @Override
    public void setSpeed(double desiredSpeed) {

    }

    @Override
    public void setPosition(double position) {

    }
}
