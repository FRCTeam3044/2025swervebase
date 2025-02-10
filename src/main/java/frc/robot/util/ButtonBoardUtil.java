package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.util.AutoTargetUtils.Reef;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.AutoTargetUtils.Reef.CoralReefLocation;

public class ButtonBoardUtil {
    private static GenericHID padOne;
    private static GenericHID padTwo;
    private static GenericHID padThree;

    private static boolean isButtonPressed(int board, int button) {
        return (board == 1 && padOne.getRawButtonPressed(button))
                || (board == 2 && padTwo.getRawButtonPressed(button))
                || (board == 3 && padThree.getRawButtonPressed(button));
    }

    public record ButtonInfo(int board, int button) {
        public boolean isPressed() {
            return isButtonPressed(board, button);
        }
    };

    public record SelectButtonInfo<T extends Enum<?>>(int board, int button, T value) {
        public boolean isPressed() {
            return isButtonPressed(board, button);
        }
    };

    private List<SelectButtonInfo<CoralReefLocation>> reefButtons = List.of(
            new SelectButtonInfo<CoralReefLocation>(0, 0, CoralReefLocation.A),
            new SelectButtonInfo<CoralReefLocation>(0, 1, CoralReefLocation.B),
            new SelectButtonInfo<CoralReefLocation>(0, 2, CoralReefLocation.C),
            new SelectButtonInfo<CoralReefLocation>(0, 3, CoralReefLocation.D),
            new SelectButtonInfo<CoralReefLocation>(0, 4, CoralReefLocation.E),
            new SelectButtonInfo<CoralReefLocation>(0, 5, CoralReefLocation.F),
            new SelectButtonInfo<CoralReefLocation>(0, 6, CoralReefLocation.G),
            new SelectButtonInfo<CoralReefLocation>(0, 7, CoralReefLocation.H),
            new SelectButtonInfo<CoralReefLocation>(0, 8, CoralReefLocation.I),
            new SelectButtonInfo<CoralReefLocation>(0, 9, CoralReefLocation.J),
            new SelectButtonInfo<CoralReefLocation>(0, 10, CoralReefLocation.K),
            new SelectButtonInfo<CoralReefLocation>(0, 11, CoralReefLocation.L));
    private ButtonInfo stationRight1 = new ButtonInfo(0, 13);
    private ButtonInfo stationRight2 = new ButtonInfo(0, 14);
    private ButtonInfo stationRight3 = new ButtonInfo(0, 15);
    private ButtonInfo stationLeft1 = new ButtonInfo(0, 16);
    private ButtonInfo stationLeft2 = new ButtonInfo(0, 17);
    private ButtonInfo stationLeft3 = new ButtonInfo(0, 18);
    private ButtonInfo processor = new ButtonInfo(0, 19);
    private List<SelectButtonInfo<CoralLevel>> levels = List.of(new SelectButtonInfo<CoralLevel>(0, 20, CoralLevel.L1),
            new SelectButtonInfo<CoralLevel>(0, 21, CoralLevel.L2),
            new SelectButtonInfo<CoralLevel>(0, 22, CoralLevel.L3),
            new SelectButtonInfo<CoralLevel>(0, 23, CoralLevel.L4));
    private ButtonInfo algaeModeToggle = new ButtonInfo(0, 24);
    private ButtonInfo net = new ButtonInfo(0, 25);
    private ButtonInfo climbUp = new ButtonInfo(0, 26);
    private ButtonInfo climbDown = new ButtonInfo(0, 27);

    private Pose2d reefPose = null;
    private CoralReefLocation reefLocation;
    private CoralLevel reefLevel;
    private Pose2d intakePose = null;
    private Pose2d processorPose = AutoTargetUtils.processor;
    private Pose2d algaeReefPose = null;

    private IntakeStation intakeStation;

    private boolean algaeMode = false;
    private boolean isProcessor = false;

    public boolean getAlgaeMode() {
        return algaeMode;
    }

    public boolean getSelectedAlgaeLocation() {
        return isProcessor;
    }

    public void periodic() {
        for (SelectButtonInfo<CoralReefLocation> button : reefButtons) {
            if (button.isPressed()) {
                reefLocation = button.value();
                reefPose = Reef.coral(reefLocation, reefLevel);
            }
        }
        for (SelectButtonInfo<CoralLevel> button : levels) {
            if (button.isPressed()) {
                reefLevel = button.value();
                reefPose = Reef.coral(reefLocation, reefLevel);
            }
        }
        // TODO: Algae Poses

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

    public CoralLevel getSelectedReefHeight() {
        return reefLevel;
    }

    public Pose2d getSelectedStationPose() {
        return intakePose;
    }

    public IntakeStation getSelectedStation() {
        return intakeStation;
    }
}
