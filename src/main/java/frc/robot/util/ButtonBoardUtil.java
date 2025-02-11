package frc.robot.util;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoTargetUtils.IntakeStations.IntakeStation;
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

    // 13 - 18
    private List<SelectButtonInfo<IntakeStation>> intakeStationButtons = List.of(
            new SelectButtonInfo<IntakeStation>(0, 13, IntakeStation.LeftOne),
            new SelectButtonInfo<IntakeStation>(0, 14, IntakeStation.LeftTwo),
            new SelectButtonInfo<IntakeStation>(0, 15, IntakeStation.LeftThree),
            new SelectButtonInfo<IntakeStation>(0, 16, IntakeStation.RightOne),
            new SelectButtonInfo<IntakeStation>(0, 17, IntakeStation.RightTwo),
            new SelectButtonInfo<IntakeStation>(0, 18, IntakeStation.RightThree));

    private ButtonInfo processor = new ButtonInfo(0, 19);
    private List<SelectButtonInfo<CoralLevel>> levels = List.of(new SelectButtonInfo<CoralLevel>(0, 20, CoralLevel.L1),
            new SelectButtonInfo<CoralLevel>(0, 21, CoralLevel.L2),
            new SelectButtonInfo<CoralLevel>(0, 22, CoralLevel.L3),
            new SelectButtonInfo<CoralLevel>(0, 23, CoralLevel.L4));
    private ButtonInfo algaeModeToggle = new ButtonInfo(0, 24);
    private ButtonInfo net = new ButtonInfo(0, 25);
    private ButtonInfo climbUp = new ButtonInfo(0, 26);
    private ButtonInfo climbDown = new ButtonInfo(0, 27);

    // Reef
    private Pose2d coralReefTargetPose = null;
    private Pose2d coralReefReferencePose = null;
    private Pose2d algaeReefTargetPose = null;
    private Pose2d algaeReefReferencePose = null;

    private CoralReefLocation coralReefLocation;
    private CoralLevel coralReefLevel;
    private Pose2d intakeStationPose = null;
    private Pose2d intakeStationReferencePose = null;

    private IntakeStation intakeStation;

    private boolean algaeMode = false;
    private boolean isProcessor = false;

    public boolean getAlgaeMode() {
        return algaeMode;
    }

    public boolean getSelectedAlgaeLocation() {
        return isProcessor;
    }

    public void periodic(Drive drive) {
        for (SelectButtonInfo<CoralReefLocation> button : reefButtons) {
            if (button.isPressed()) {
                coralReefLocation = button.value();
                coralReefTargetPose = Reef.coral(coralReefLocation, coralReefLevel);
                coralReefReferencePose = coralReefLocation.pose();
                algaeReefTargetPose = Reef.algae(coralReefLocation.algae());
                algaeReefReferencePose = coralReefLocation.algae().pose();
            }
        }
        for (SelectButtonInfo<CoralLevel> button : levels) {
            if (button.isPressed()) {
                coralReefLevel = button.value();
                coralReefTargetPose = Reef.coral(coralReefLocation, coralReefLevel);
            }
        }

        for (SelectButtonInfo<IntakeStation> button : intakeStationButtons) {
            if (button.isPressed()) {
                intakeStation = button.value();
                intakeStationPose = intakeStation.pose();
                intakeStationReferencePose = intakeStation.pose();
            }
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

        Logger.recordOutput("Dist to coral reef position",
                AutoTargetUtils.robotDistToPose(drive, coralReefReferencePose));
        Logger.recordOutput("Dist to algae reef position",
                AutoTargetUtils.robotDistToPose(drive, algaeReefReferencePose));
        Logger.recordOutput("Dist to intake station position",
                AutoTargetUtils.robotDistToPose(drive, intakeStationReferencePose));
        Logger.recordOutput("ButtonBoard/AlgaeMode", algaeMode);
        Logger.recordOutput("ButtonBoard/Processor", isProcessor);
        Logger.recordOutput("ButtonBoard/CoralReefLocation", coralReefLocation);
        Logger.recordOutput("ButtonBoard/CoralReefLevel", coralReefLevel);
        Logger.recordOutput("ButtonBoard/IntakeStation", intakeStation);
        Logger.recordOutput("ButtonBoard/ClimbUp", climbUp.isPressed());
        Logger.recordOutput("ButtonBoard/ClimbDown", climbDown.isPressed());
    }

    public Pose2d getCoralReefTarget() {
        return coralReefTargetPose;
    }

    public DoubleSupplier getCoralReefTargetDist(Drive drive) {
        return () -> AutoTargetUtils.robotDistToPose(drive, coralReefReferencePose);
    }

    public Pose2d getAlgaeReefTarget() {
        return algaeReefTargetPose;
    }

    public DoubleSupplier getAlgaeReefTargetDist(Drive drive) {
        return () -> AutoTargetUtils.robotDistToPose(drive, algaeReefReferencePose);
    }

    public Pose2d getIntakeStationTarget() {
        return intakeStationPose;
    }

    public DoubleSupplier getIntakeStationTargetDist(Drive drive) {
        return () -> AutoTargetUtils.robotDistToPose(drive, intakeStationReferencePose);
    }

    public boolean isProcessor() {
        return isProcessor;
    }
}
