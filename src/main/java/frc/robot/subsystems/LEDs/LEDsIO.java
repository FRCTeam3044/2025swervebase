package frc.robot.subsystems.LEDs;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern;

public interface LEDsIO {
    @AutoLog
    public static class LEDsIOInputs {
    }

    public default void setSolidColor(LEDPattern color){};
    public default void setBlinkingColor(LEDPattern color){};
    public default void setSpinningColor(LEDPattern color){};
}
