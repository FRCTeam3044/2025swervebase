package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
    private final AnalogInput proximitySensor = new AnalogInput(proximityChannel);
    Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private boolean isCurrentSpiking;

    public EndEffectorIOSpark() {
        tryUntilOk(motor, 5, () -> motor.configure(EndEffectorConfig.motorConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        isCurrentSpiking = checkCurrentSpike();
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);

        ifOk(
                motor,
                new DoubleSupplier[] { motor::getAppliedOutput, motor::getBusVoltage },
                (values) -> inputs.appliedVoltage = values[0] * values[1]);

        inputs.hasCoral = hasCoral();
        inputs.hasAlgae = hasAlgae();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @AutoLogOutput(key = "IsCurrentSpiking")
    private boolean checkCurrentSpike() {
        isCurrentSpiking = motor.getOutputCurrent() > currentThreshold.get();
        return debouncer.calculate(isCurrentSpiking);
    }

    public double readSensor() {
        double v = proximitySensor.getAverageVoltage();
        // Freaky magic numbers to convert to cm (maybe)
        double d = 26.449 * (Math.pow(v, -1.226));
        return d;
    }

    private boolean hasCoral() {
        return isCurrentSpiking && readSensor() < coralProximityDistance.get();
    }

    private boolean hasAlgae() {
        return isCurrentSpiking && readSensor() > coralProximityDistance.get();
    }
}
