package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.util.AutoTargetUtils.Reef;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.AutoTargetUtils.Reef.CoralReefLocation;

public class ButtonBoardUtil {
    private static GenericHID padOne;
    private static GenericHID padTwo;
    private static GenericHID padThree;

    public record ButtonInfo(int board, int button) {
        public boolean isPressed() {
            return (board == 1 && ButtonBoardUtil.padOne.getRawButtonPressed(button))
                    || (board == 2 && ButtonBoardUtil.padTwo.getRawButtonPressed(button))
                    || (board == 3 && ButtonBoardUtil.padThree.getRawButtonPressed(button));
        }
    };

    private ButtonInfo reefA = new ButtonInfo(0, 0);
    private ButtonInfo reefB = new ButtonInfo(0, 1);
    private ButtonInfo reefC = new ButtonInfo(0, 2);
    private ButtonInfo reefD = new ButtonInfo(0, 3);
    private ButtonInfo reefE = new ButtonInfo(0, 4);
    private ButtonInfo reefF = new ButtonInfo(0, 5);
    private ButtonInfo reefG = new ButtonInfo(0, 6);
    private ButtonInfo reefH = new ButtonInfo(0, 7);
    private ButtonInfo reefI = new ButtonInfo(0, 8);
    private ButtonInfo reefJ = new ButtonInfo(0, 9);
    private ButtonInfo reefK = new ButtonInfo(0, 10);
    private ButtonInfo reefL = new ButtonInfo(0, 11);
    private ButtonInfo stationRight1 = new ButtonInfo(0, 13);
    private ButtonInfo stationRight2 = new ButtonInfo(0, 14);
    private ButtonInfo stationRight3 = new ButtonInfo(0, 15);
    private ButtonInfo stationLeft1 = new ButtonInfo(0, 16);
    private ButtonInfo stationLeft2 = new ButtonInfo(0, 17);
    private ButtonInfo stationLeft3 = new ButtonInfo(0, 18);
    private ButtonInfo processor = new ButtonInfo(0, 19);
    private ButtonInfo levelOne = new ButtonInfo(0, 20);
    private ButtonInfo levelTwo = new ButtonInfo(0, 21);
    private ButtonInfo levelThree = new ButtonInfo(0, 22);
    private ButtonInfo levelFour = new ButtonInfo(0, 23);
    private ButtonInfo algaeModeToggle = new ButtonInfo(0, 24);
    private ButtonInfo net = new ButtonInfo(0, 25);
    private ButtonInfo climbUp = new ButtonInfo(0, 26);
    private ButtonInfo climbDown = new ButtonInfo(0, 27);

    private Pose2d reefPose = null;
    private Pose2d intakePose = null;
    private Pose2d processorPose = AutoTargetUtils.processor;
    private Pose2d algaeReefPose = null;

    private IntakeStation intakeStation;
    private CoralLevel reefLevel;

    private boolean algaeMode = false;
    private boolean isProcessor = false;

    public boolean getAlgaeMode() {
        return algaeMode;
    }

    public boolean getSelectedAlgaeLocation() {
        return isProcessor;
    }

    public void periodic() {
        if (reefA.isPressed()) {
            reefPose = Reef.coral(CoralReefLocation.A, reefLevel);
        }
        if (reefB.isPressed()) {
            reefPose = AutoTargetUtils.reefB();
        }
        if (reefC.isPressed()) {
            reefPose = AutoTargetUtils.reefC();
        }
        if (reefD.isPressed()) {
            reefPose = AutoTargetUtils.reefD();
        }
        if (reefE.isPressed()) {
            reefPose = AutoTargetUtils.reefE();
        }
        if (reefF.isPressed()) {
            reefPose = AutoTargetUtils.reefF();
        }
        if (reefG.isPressed()) {
            reefPose = AutoTargetUtils.reefG();
        }
        if (reefH.isPressed()) {
            reefPose = AutoTargetUtils.reefH();
        }
        if (reefI.isPressed()) {
            reefPose = AutoTargetUtils.reefI();
        }
        if (reefJ.isPressed()) {
            reefPose = AutoTargetUtils.reefJ();
        }
        if (reefK.isPressed()) {
            reefPose = AutoTargetUtils.reefK();
        }
        if (reefL.isPressed()) {
            reefPose = AutoTargetUtils.reefL();
        }
        if (stationLeft1.isPressed()) {
            intakePose = AutoTargetUtils.leftStation1();
            intakeStation = IntakeStation.L1;
        }
        if (stationLeft2.isPressed()) {
            intakePose = AutoTargetUtils.leftStation2();
            intakeStation = IntakeStation.L2;
        }
        if (stationLeft3.isPressed()) {
            intakePose = AutoTargetUtils.leftStation3();
            intakeStation = IntakeStation.L3;
        }
        if (stationRight1.isPressed()) {
            intakePose = AutoTargetUtils.rightStation1();
            intakeStation = IntakeStation.R1;
        }
        if (stationRight2.isPressed()) {
            intakePose = AutoTargetUtils.rightStation2();
            intakeStation = IntakeStation.R2;
        }
        if (stationRight3.isPressed()) {
            intakePose = AutoTargetUtils.rightStation3();
            intakeStation = IntakeStation.R3;
        }
        if (reefA.isPressed() || reefB.isPressed()) {
            algaeReefPose = AutoTargetUtils.algaeA();
        }
        if (reefC.isPressed() || reefD.isPressed()) {
            algaeReefPose = AutoTargetUtils.algaeB();
        }
        if (reefE.isPressed() || reefF.isPressed()) {
            algaeReefPose = AutoTargetUtils.algaeC;
        }
        if (reefG.isPressed() || reefH.isPressed()) {
            algaeReefPose = AutoTargetUtils.algaeD;
        }
        if (reefI.isPressed() || reefJ.isPressed()) {
            algaeReefPose = AutoTargetUtils.algaeE;
        }
        if (reefK.isPressed() || reefL.isPressed()) {
            algaeReefPose = AutoTargetUtils.algaeF;
        }
        if (levelOne.isPressed()) {
            reefLevel = CoralLevel.L1;
        }
        if (levelTwo.isPressed()) {
            reefLevel = CoralLevel.L2;
        }
        if (levelThree.isPressed()) {
            reefLevel = CoralLevel.L3;
        }
        if (levelFour.isPressed()) {
            reefLevel = CoralLevel.L4;
        }
        if (algaeModeToggle.isPressed()) {
            algaeMode = !algaeMode;
        }
        if (processor.isPressed()) {
            isProcessor = true;
        }
        if (net.isPressed()) {
            isProcessor = false;
        }
    }

    public enum IntakeStation {
        L1,
        L2,
        L3,
        R1,
        R2,
        R3
    }

    public Pose2d getSelectedReef() {
        return reefPose;
    }

    public Pose2d getSelecectedAlgae() {
        return algaeReefPose;
    }

    public ReefHeight getSelectedReefHeight() {
        return reefLevel;
    }

    public Pose2d getSelectedStationPose() {
        return intakePose;
    }

    public IntakeStation getSelectedStation() {
        return intakeStation;
    }
}
