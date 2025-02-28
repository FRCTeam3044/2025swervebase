package frc.robot.util.bboard;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoTargetUtils.IntakeStations.IntakeStation;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoTargetUtils;
import frc.robot.util.AllianceUtil.AllianceColor;
import frc.robot.util.AutoTargetUtils.IntakeStations;
import frc.robot.util.AutoTargetUtils.Reef;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.AutoTargetUtils.Reef.CoralReefLocation;
import me.nabdev.oxconfig.ConfigurableParameter;

public class ButtonBoard {
    public enum ManualMode {
        AUTO, SEMI, MANUAL
    }

    private final BBoardIO boardIO;

    public ButtonBoard(BBoardIO boardIO) {
        this.boardIO = boardIO;
    }

    public record ButtonInfo(int board, int button) {
    };

    public record SelectButtonInfo<T extends Enum<?>>(int board, int button, T value) {
        public ButtonInfo buttonInfo() {
            return new ButtonInfo(board, button);
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

    private ButtonInfo extraOne = new ButtonInfo(2, 1);
    private ButtonInfo extraTwo = new ButtonInfo(2, 2);
    private ButtonInfo extraThree = new ButtonInfo(2, 3);
    private ButtonInfo extraFour = new ButtonInfo(2, 4);

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
    private boolean isNet = false;

    private ManualMode manualMode = ManualMode.AUTO;
    private boolean manualIdle = false;
    private boolean manualIntake = false;

    public boolean getAlgaeMode() {
        return algaeMode;
    }

    public boolean getSelectedAlgaeLocation() {
        return isProcessor;
    }

    private AllianceColor lastAllianceColor = AllianceColor.UNKNOWN;
    private boolean intakeJustChanged = false;
    private boolean coralReefJustChanged = false;

    public void periodic(Drive drive) {
        intakeJustChanged = false;
        coralReefJustChanged = false;
        if (lastAllianceColor != AllianceUtil.getAlliance()) {
            if (coralReefLevel != null && coralReefLocation != null) {
                coralReefTargetPose = Reef.coral(coralReefLocation, coralReefLevel);
                coralReefReferencePose = coralReefLocation.pose();
            }
            if (coralReefLocation != null) {
                algaeReefTargetPose = Reef.algae(coralReefLocation.algae());
                algaeReefReferencePose = coralReefLocation.algae().pose();
            }
            if (intakeStation != null) {
                intakeStationPose = IntakeStations.intakeStation(intakeStation);
                intakeStationReferencePose = intakeStation.pose();
            }
        }
        boardIO.periodic();
        for (SelectButtonInfo<CoralReefLocation> button : reefButtons) {
            if (boardIO.isPressed(button)) {
                coralReefJustChanged = true;
                coralReefLocation = button.value();
                coralReefReferencePose = coralReefLocation.pose();
                algaeReefTargetPose = Reef.algae(coralReefLocation.algae());
                algaeReefReferencePose = coralReefLocation.algae().pose();

                if (coralReefLevel != null)
                    coralReefTargetPose = Reef.coral(coralReefLocation, coralReefLevel);
            }
        }
        for (SelectButtonInfo<CoralLevel> button : levels) {
            if (boardIO.isPressed(button)) {
                if (manualMode == ManualMode.MANUAL) {
                    manualIdle = false;
                }
                coralReefLevel = button.value();
                if (coralReefLocation != null)
                    coralReefTargetPose = Reef.coral(coralReefLocation, coralReefLevel);
            }
        }

        for (SelectButtonInfo<IntakeStation> button : intakeStationButtons) {
            if (boardIO.isPressed(button)) {
                intakeJustChanged = true;
                intakeStation = button.value();
                intakeStationPose = IntakeStations.intakeStation(intakeStation);
                intakeStationReferencePose = intakeStation.pose();
            }
        }
        if (boardIO.isPressed(algaeModeToggle)) {
            algaeMode = !algaeMode;
        }
        if (boardIO.isPressed(processor)) {
            isProcessor = true;
        }
        if (boardIO.isPressed(net)) {
            isNet = false;
        }
        if (boardIO.isPressed(extraOne) && boardIO.isPressed(extraThree) && boardIO.isPressed(extraFour)) {
            if (boardIO.isPressed(extraTwo)) {
                if (manualMode == ManualMode.MANUAL) {
                    manualMode = ManualMode.AUTO;
                } else {
                    manualMode = ManualMode.MANUAL;
                }
            } else {
                if (manualMode == ManualMode.SEMI) {
                    manualMode = ManualMode.AUTO;
                } else {
                    manualMode = ManualMode.SEMI;
                    manualIdle = true;
                    manualIntake = false;
                }
            }
        }
        if (manualMode == ManualMode.SEMI && boardIO.isPressed(intakeStationButtons.get(5))) {
            manualIdle = true;
            manualIntake = false;
        }
        if (manualMode == ManualMode.SEMI && boardIO.isPressed(intakeStationButtons.get(3))) {
            manualIdle = false;
            manualIntake = true;
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
        Logger.recordOutput("ButtonBoard/ClimbUp", boardIO.isPressed(climbUp));
        Logger.recordOutput("ButtonBoard/ClimbDown", boardIO.isPressed(climbDown));
        Logger.recordOutput("ButtonBoard/ManualMode", manualMode);

        // Log to SmartDashboard for Button Board LED controller
        SmartDashboard.putBoolean("ButtonBoard/AlgaeMode", algaeMode);
        SmartDashboard.putBoolean("ButtonBoard/isProcessor", isProcessor);
        SmartDashboard.putBoolean("ButtonBoard/isNet", isNet);
        SmartDashboard.putBoolean("ButtonBoard/ClimbUp", boardIO.isPressed(climbUp));
        SmartDashboard.putBoolean("ButtonBoard/ClimbDown", boardIO.isPressed(climbDown));
        if (coralReefLocation != null) {
            SmartDashboard.putString("ButtonBoard/CoralReefLocation", coralReefLocation.toString());
        } else {
            SmartDashboard.putString("ButtonBoard/CoralReefLocation", "null");
        }
        if (coralReefLevel != null) {
            SmartDashboard.putString("ButtonBoard/CoralReefLevel", coralReefLevel.toString());
        } else {
            SmartDashboard.putString("ButtonBoard/CoralReefLevel", "null");
        }
        if (intakeStation != null) {
            SmartDashboard.putString("ButtonBoard/IntakeStation", intakeStation.toString());
        } else {
            SmartDashboard.putString("ButtonBoard/IntakeStation", "null");
        }
    }

    public Pose2d getCoralReefTarget() {
        return coralReefTargetPose;
    }

    public CoralLevel getCoralReefLevel() {
        return coralReefLevel;
    }

    public DoubleSupplier getCoralReefReferenceDist(Drive drive) {
        return () -> {
            if (coralReefReferencePose == null) {
                return Double.MAX_VALUE;
            }
            return AutoTargetUtils.robotDistToPose(drive, coralReefReferencePose);
        };
    }

    public DoubleSupplier getCoralReefTargetDist(Drive drive) {
        return () -> {
            if (coralReefTargetPose == null) {
                return Double.MAX_VALUE;
            }
            return AutoTargetUtils.robotDistToPose(drive, coralReefTargetPose);
        };
    }

    public Pose2d getAlgaeReefTarget() {
        return algaeReefTargetPose;
    }

    public DoubleSupplier getAlgaeReefReferenceDist(Drive drive) {
        return () -> {
            if (algaeReefReferencePose == null) {
                return Double.MAX_VALUE;
            }
            return AutoTargetUtils.robotDistToPose(drive, algaeReefReferencePose);
        };
    }

    public Pose2d getIntakeStationTarget() {
        return intakeStationPose;
    }

    public DoubleSupplier getIntakeStationReferenceDist(Drive drive) {
        return () -> {
            if (intakeStationReferencePose == null) {
                return Double.MAX_VALUE;
            }
            return AutoTargetUtils.robotDistToPose(drive, intakeStationReferencePose);
        };
    }

    public DoubleSupplier getIntakeStationTargetDist(Drive drive) {
        return () -> {
            if (intakeStationPose == null) {
                return Double.MAX_VALUE;
            }
            return AutoTargetUtils.robotDistToPose(drive, intakeStationPose);
        };
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
                return AutoTargetUtils.robotDistToPose(drive, AutoTargetUtils.processor()) < processorDistThreshold
                        .get();
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

    public boolean intakeSelected() {
        if (algaeMode) {
            return algaeReefTargetPose != null;
        } else {
            return intakeStationPose != null;
        }
    }

    public boolean scoringSelected() {
        if (algaeMode) {
            return true;
        } else {
            return coralReefTargetPose != null;
        }
    }

    public boolean intakeJustChanged() {
        return intakeJustChanged;
    }

    public boolean coralReefJustChanged() {
        return coralReefJustChanged;
    }

    public boolean fullManual() {
        return manualMode == ManualMode.MANUAL;
    }

    public boolean semiAuto() {
        return manualMode == ManualMode.SEMI;
    }

    public boolean fullAuto() {
        return manualMode == ManualMode.AUTO;
    }

    public boolean idleInManual() {
        return manualIdle;
    }

    public boolean intakeInManual() {
        return manualIntake;
    }
}
