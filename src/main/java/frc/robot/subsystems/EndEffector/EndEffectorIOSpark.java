package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogInput;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
    private final AnalogInput proximitySensor = new AnalogInput(proximityChannel);

    public EndEffectorIOSpark() {
        tryUntilOk(motor, 5, () -> motor.configure(EndEffectorConfig.motorConfig, 
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.coralCurrentAmps = value);
      
        ifOk(
                motor,
                new DoubleSupplier[] { motor::getAppliedOutput, motor::getBusVoltage },
                (values) -> inputs.coralAppliedVoltage = values[0] * values[1]);
    }

    @Override
    public void setCoralSpeed(double speed) {
        motor.set(speed);
    }

    public double readSensor() {
        double v = proximitySensor.getAverageVoltage();
        double d = 26.449*(Math.pow(v,-1.226));
        return d;
    }

    public boolean hasCoral() {
        if(readSensor() < coralProximityDistance.get() && motor.getOutputCurrent() > 1) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean hasAlgae() {
        if(readSensor() > coralProximityDistance.get() && motor.getOutputCurrent() > 1) {
            return true;
        }
        else {
            return false;
        }
    }
}
