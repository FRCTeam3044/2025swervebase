package frc.robot.util;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoTargetUtils.IntakeStations.IntakeStation;
import frc.robot.util.AutoTargetUtils.IntakeStations;
import frc.robot.util.AutoTargetUtils.Reef;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.AutoTargetUtils.Reef.CoralReefLocation;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ButtonBoardUtil {
    private static GenericHID padOne = new GenericHID(2);
    private static GenericHID padTwo = new GenericHID(3);
    private static GenericHID padThree = new GenericHID(4);

    private static boolean isButtonPressed(int board, int button) {
        return (board == 0 && padOne.getRawButtonPressed(button))
                || (board == 1 && padTwo.getRawButtonPressed(button))
                || (board == 2 && padThree.getRawButtonPressed(button));
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
            new SelectButtonInfo<CoralReefLocation>(0, 1, CoralReefLocation.A),
            new SelectButtonInfo<CoralReefLocation>(0, 2, CoralReefLocation.B),
            new SelectButtonInfo<CoralReefLocation>(0, 3, CoralReefLocation.C),
            new SelectButtonInfo<CoralReefLocation>(0, 4, CoralReefLocation.D),
            new SelectButtonInfo<CoralReefLocation>(0, 5, CoralReefLocation.E),
            new SelectButtonInfo<CoralReefLocation>(0, 6, CoralReefLocation.F),
            new SelectButtonInfo<CoralReefLocation>(0, 7, CoralReefLocation.G),
            new SelectButtonInfo<CoralReefLocation>(0, 8, CoralReefLocation.H),
            new SelectButtonInfo<CoralReefLocation>(0, 9, CoralReefLocation.I),
            new SelectButtonInfo<CoralReefLocation>(0, 10, CoralReefLocation.J),
            new SelectButtonInfo<CoralReefLocation>(0, 11, CoralReefLocation.K),
            new SelectButtonInfo<CoralReefLocation>(0, 12, CoralReefLocation.L));

    // 13 - 18
    private List<SelectButtonInfo<IntakeStation>> intakeStationButtons = List.of(
            new SelectButtonInfo<IntakeStation>(1, 1, IntakeStation.LeftOne),
            new SelectButtonInfo<IntakeStation>(1, 2, IntakeStation.LeftTwo),
            new SelectButtonInfo<IntakeStation>(1, 3, IntakeStation.LeftThree),
            new SelectButtonInfo<IntakeStation>(1, 7, IntakeStation.RightOne),
            new SelectButtonInfo<IntakeStation>(1, 6, IntakeStation.RightTwo),
            new SelectButtonInfo<IntakeStation>(1, 5, IntakeStation.RightThree));

    private ButtonInfo processor = new ButtonInfo(2, 12);
    private List<SelectButtonInfo<CoralLevel>> levels = List.of(new SelectButtonInfo<CoralLevel>(2, 10, CoralLevel.L1),
            new SelectButtonInfo<CoralLevel>(2, 9, CoralLevel.L2),
            new SelectButtonInfo<CoralLevel>(2, 8, CoralLevel.L3),
            new SelectButtonInfo<CoralLevel>(2, 7, CoralLevel.L4));
    private ButtonInfo algaeModeToggle = new ButtonInfo(1, 4);
    private ButtonInfo net = new ButtonInfo(0, 11);
    private ButtonInfo climbUp = new ButtonInfo(0, 5);
    private ButtonInfo climbDown = new ButtonInfo(0, 6);

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
                coralReefReferencePose = coralReefLocation.pose();
                algaeReefTargetPose = Reef.algae(coralReefLocation.algae());
                algaeReefReferencePose = coralReefLocation.algae().pose();

                if (coralReefLevel != null)
                    coralReefTargetPose = Reef.coral(coralReefLocation, coralReefLevel);
            }
        }
        for (SelectButtonInfo<CoralLevel> button : levels) {
            if (button.isPressed()) {
                coralReefLevel = button.value();
                if (coralReefLocation != null)
                    coralReefTargetPose = Reef.coral(coralReefLocation, coralReefLevel);
            }
        }

        for (SelectButtonInfo<IntakeStation> button : intakeStationButtons) {
            if (button.isPressed()) {
                intakeStation = button.value();
                intakeStationPose = IntakeStations.intakeStation(intakeStation);
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

        if (coralReefReferencePose != null)
            Logger.recordOutput("Dist to coral reef position",
                    AutoTargetUtils.robotDistToPose(drive, coralReefReferencePose));
        if (algaeReefReferencePose != null)
            Logger.recordOutput("Dist to algae reef position",
                    AutoTargetUtils.robotDistToPose(drive, algaeReefReferencePose));
        if (intakeStationReferencePose != null)
            Logger.recordOutput("Dist to intake station position",
                    AutoTargetUtils.robotDistToPose(drive, intakeStationReferencePose));
        Logger.recordOutput("ButtonBoard/AlgaeMode", algaeMode);
        Logger.recordOutput("ButtonBoard/Processor", isProcessor);
        Logger.recordOutput("ButtonBoard/CoralReefLocation", coralReefLocation);
        Logger.recordOutput("ButtonBoard/CoralReefPose", coralReefTargetPose);
        Logger.recordOutput("ButtonBoard/CoralReefLevel", coralReefLevel);
        Logger.recordOutput("ButtonBoard/IntakeStation", intakeStation);
        Logger.recordOutput("ButtonBoard/IntakeStationPose", intakeStationPose);
        Logger.recordOutput("ButtonBoard/ClimbUp", climbUp.isPressed());
        Logger.recordOutput("ButtonBoard/ClimbDown", climbDown.isPressed());
    }

    public Pose2d getCoralReefTarget() {
        return coralReefTargetPose;
    }

    public CoralLevel getCoralReefLevel() {
        return coralReefLevel;
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

    public ConfigurableParameter<Double> processorDistThreshold = new ConfigurableParameter<Double>(0.5,
            "Processor score dist threshold");
    public ConfigurableParameter<Double> coralReefDistThreshold = new ConfigurableParameter<Double>(0.5,
            "Coral reef score dist threshold");
    public ConfigurableParameter<Double> netDistThreshold = new ConfigurableParameter<Double>(0.5,
            "Net score dist threshold");

    public boolean closeToScoringTarget(Drive drive) {
        if (algaeMode) {
            if (isProcessor) {
                if (coralReefTargetPose == null) {
                    return false;
                }
                return AutoTargetUtils.robotDistToPose(drive, algaeReefTargetPose) < processorDistThreshold.get();
            } else {
                // TODO: Net
                return false;
            }
        } else {
            if (coralReefTargetPose == null) {
                return false;
            }
            return AutoTargetUtils.robotDistToPose(drive, coralReefTargetPose) < coralReefDistThreshold.get();
        }
    }

    public ConfigurableParameter<Double> algaeReefDistThreshold = new ConfigurableParameter<>(0.5,
            "Algae intake reef dist threshold");
    public ConfigurableParameter<Double> intakeStationDistThreshold = new ConfigurableParameter<>(0.5,
            "Intake station dist threshold");

    public boolean closeToIntakeTarget(Drive drive) {
        if (algaeMode) {
            if (algaeReefTargetPose == null) {
                return false;
            }
            return AutoTargetUtils.robotDistToPose(drive, algaeReefTargetPose) < algaeReefDistThreshold.get();
        } else {
            if (intakeStationPose == null) {
                return false;
            }

            return AutoTargetUtils.robotDistToPose(drive, intakeStationPose) < intakeStationDistThreshold.get();
        }
    }
}
