package frc.robot.util.bboard;

import frc.robot.util.bboard.ButtonBoard.ButtonInfo;
import frc.robot.util.bboard.ButtonBoard.SelectButtonInfo;

public interface BBoardIO {
    public boolean isPressed(ButtonInfo button);
    public boolean isPressed(SelectButtonInfo<?> button);
    public default void periodic(){};
}
