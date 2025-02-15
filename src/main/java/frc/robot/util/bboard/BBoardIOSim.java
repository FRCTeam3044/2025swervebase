package frc.robot.util.bboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.bboard.ButtonBoard.ButtonInfo;
import frc.robot.util.bboard.ButtonBoard.SelectButtonInfo;

public class BBoardIOSim implements BBoardIO{
    public BBoardIOSim() {
        SmartDashboard.putNumberArray("ButtonBoardSim/Pressed", new double[0]);
    }

    private ArrayList<ButtonInfo> pressedButtons = new ArrayList<>();

    public void periodic(){
        double[] pressed = SmartDashboard.getNumberArray("ButtonBoardSim/Pressed", new double[0]);
        pressedButtons.clear();
        for (int i = 0; i < pressed.length - 1; i += 2) {
            pressedButtons.add(new ButtonInfo((int) pressed[i], (int) pressed[i + 1]));
        }
        SmartDashboard.putNumberArray("ButtonBoardSim/Pressed", new double[0]);
    }

    @Override
    public boolean isPressed(ButtonInfo button) {
        return pressedButtons.contains(button);
    }

    @Override
    public boolean isPressed(SelectButtonInfo<?> button) {
        return pressedButtons.contains(button.buttonInfo());
    }
    
}
