package frc.robot.subsystems.endEffector;

import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import static frc.robot.subsystems.endEffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
    Debouncer debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private DigitalInput coralSwitch = new DigitalInput(2);
    private RelativeEncoder encoder = motor.getEncoder();

    public EndEffectorIOSpark() {
        tryUntilOk(motor, 5, () -> motor.configure(EndEffectorConfig.motorConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
        ifOk(motor, encoder::getVelocity, (value) -> inputs.velocity = value);

        ifOk(
                motor,
                new DoubleSupplier[] { motor::getAppliedOutput, motor::getBusVoltage },
                (values) -> inputs.appliedVoltage = values[0] * values[1]);

        inputs.wheelsStuck = checkWheelsStuck(inputs);
        inputs.limitSwitchPressed = limitSwitchPressed();

        inputs.hasCoral = hasCoral();
        inputs.hasAlgae = hasAlgae(inputs);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @AutoLogOutput(key = "WheelsAreStuck")
    private boolean checkWheelsStuck(EndEffectorIOInputs inputs) {
        boolean stuck = Math.abs(inputs.appliedVoltage) > voltageThreshold.get()
                && Math.abs(inputs.velocity) < speedThreshold.get();
        return debouncer.calculate(stuck);
    }

    private boolean hasCoral() {
        return limitSwitchPressed();
    }

    private boolean hasAlgae(EndEffectorIOInputs inputs) {
        return inputs.wheelsStuck && !limitSwitchPressed();
    }

    private boolean limitSwitchPressed() {
        return !coralSwitch.get();
    }
}
