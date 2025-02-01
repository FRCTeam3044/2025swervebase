package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public interface LEDsIO {

    public default void setSolidColor(LEDPattern color){};
    public default void setBlinkingColor(Color color){};
    public default void setSpinningColor(Color color1, Color color2){};
}
