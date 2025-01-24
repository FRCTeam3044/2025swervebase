package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ButtonBoardUtil {
    public record ButtonInfo(boolean board1, int button){};

    private ButtonInfo reefA = new ButtonInfo(false, 0);
    private ButtonInfo reefB = new ButtonInfo(false, 1);
    private ButtonInfo reefC = new ButtonInfo(false, 2);
    private ButtonInfo reefD = new ButtonInfo(false, 3);
    private ButtonInfo reefE = new ButtonInfo(false, 4);
    private ButtonInfo reefF = new ButtonInfo(false, 5);
    private ButtonInfo reefG = new ButtonInfo(false, 6);
    private ButtonInfo reefH = new ButtonInfo(false, 7);
    private ButtonInfo reefI = new ButtonInfo(false, 8);
    private ButtonInfo reefJ = new ButtonInfo(false, 9);
    private ButtonInfo reefK = new ButtonInfo(false, 10);
    private ButtonInfo reefL = new ButtonInfo(false,11);
    private ButtonInfo stationRight1 = new ButtonInfo(false, 13);
    private ButtonInfo stationRight2 = new ButtonInfo(false, 14);
    private ButtonInfo stationRight3 = new ButtonInfo(false, 15);
    private ButtonInfo stationLeft1 = new ButtonInfo(false, 16);
    private ButtonInfo stationLeft2 = new ButtonInfo(false, 17);
    private ButtonInfo stationLeft3 = new ButtonInfo(false, 18);
    private ButtonInfo processor = new ButtonInfo(false, 19);

    public enum ReefHeight {
        L1,
        L2,
        L3,
        L4,
    }

    public enum IntakeStation {
        L1,
        L2,
        L3,
        R1,
        R2,
        R3
    }

    public Pose2d getSelectedReef(){
        return null;
    }

    public ReefHeight getSelectedReefHeight(){
        return null;
    }

    public Pose2d getSelectedStationPose(){
        return null;
    }

    public IntakeStation getSelectedStation(){
        return null;
    }
}
