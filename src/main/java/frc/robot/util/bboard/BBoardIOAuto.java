package frc.robot.util.bboard;

import java.util.List;

import frc.robot.util.bboard.ButtonBoard.ButtonInfo;
import frc.robot.util.bboard.ButtonBoard.SelectButtonInfo;

public class BBoardIOAuto implements BBoardIO {
    private List<SelectButtonInfo<?>> selectPressed = List.of();
    private List<ButtonInfo> pressed = List.of();

    @Override
    public boolean isPressed(ButtonInfo button) {
        return pressed.contains(button);
    }

    @Override
    public boolean isBeingPressed(ButtonInfo button) {
        return pressed.contains(button);
    }

    @Override
    public boolean isPressed(SelectButtonInfo<?> button) {
        return selectPressed.contains(button);
    }

    @Override
    public boolean isBeingPressed(SelectButtonInfo<?> button) {
        return selectPressed.contains(button);
    }

    public void setSelectPressed(List<SelectButtonInfo<?>> pressed) {
        this.selectPressed = pressed;
    }

    public void setPressed(List<ButtonInfo> pressed) {
        this.pressed = pressed;
    }
}
