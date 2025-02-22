package frc.robot.util.bboard;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.util.bboard.ButtonBoard.ButtonInfo;
import frc.robot.util.bboard.ButtonBoard.SelectButtonInfo;

public class BBoardIOReal implements BBoardIO {
    private static GenericHID padOne = new GenericHID(2);
    private static GenericHID padTwo = new GenericHID(3);
    private static GenericHID padThree = new GenericHID(4);

    @Override
    public boolean isPressed(ButtonInfo buttonInfo) {
        int board = buttonInfo.board();
        int button = buttonInfo.button();
        return (board == 0 && padOne.getRawButtonPressed(button))
                || (board == 1 && padTwo.getRawButtonPressed(button))
                || (board == 2 && padThree.getRawButtonPressed(button));
    }

    
    @Override
    public boolean isPressed(SelectButtonInfo<?> buttonInfo) {
        int board = buttonInfo.board();
        int button = buttonInfo.button();
        return (board == 0 && padOne.getRawButtonPressed(button))
                || (board == 1 && padTwo.getRawButtonPressed(button))
                || (board == 2 && padThree.getRawButtonPressed(button));
    }
}
