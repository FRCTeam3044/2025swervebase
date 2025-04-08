package frc.robot;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.util.AutoTargetUtils.IntakeStations.IntakeStation;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.AutoTargetUtils.Reef.CoralReefLocation;
import frc.robot.util.bboard.ButtonBoard;
import frc.robot.util.bboard.ButtonBoard.ButtonInfo;
import frc.robot.util.bboard.ButtonBoard.SelectButtonInfo;
import me.nabdev.oxconfig.ConfigurableParameter;

public class Autos {
    private final ButtonBoard board;
    private final EndEffector endEffector;
    public final LoggedDashboardChooser<List<AutoStep>> autoChooser;

    private Debouncer hasCoralDebouncer = new Debouncer(0.2, DebounceType.kFalling);
    private ConfigurableParameter<Double> algaeMaxIntakeTime = new ConfigurableParameter<>(2.0,
            "Algae Max Intake Time (s)");

    public Autos(ButtonBoard board, EndEffector endEffector) {
        this.board = board;
        this.endEffector = endEffector;

        autoChooser = new LoggedDashboardChooser<>("Auto Routine");
        autoChooser.addDefaultOption("None", List.of());
        autoChooser.addOption("Right L4", List.of(
                scoreCoral(CoralReefLocation.E, CoralLevel.L4, IntakeStation.RightTwo),
                scoreCoral(CoralReefLocation.D, CoralLevel.L4, IntakeStation.RightTwo),
                scoreCoral(CoralReefLocation.C, CoralLevel.L4, IntakeStation.RightTwo),
                scoreCoral(CoralReefLocation.B, CoralLevel.L4, IntakeStation.RightTwo)));
        autoChooser.addOption("Left L4", List.of(
                scoreCoral(CoralReefLocation.J, CoralLevel.L4, IntakeStation.LeftTwo),
                scoreCoral(CoralReefLocation.K, CoralLevel.L4, IntakeStation.LeftTwo),
                scoreCoral(CoralReefLocation.L, CoralLevel.L4, IntakeStation.LeftTwo),
                scoreCoral(CoralReefLocation.A, CoralLevel.L4, IntakeStation.LeftTwo)));
        autoChooser.addOption("Back Left", List.of(
                singleCoral(CoralReefLocation.H, CoralLevel.L4)));
        autoChooser.addOption("Back Right", List.of(
                singleCoral(CoralReefLocation.G, CoralLevel.L4)));
        autoChooser.addOption("Left Algae", List.of(
                singleCoralIntakeAlgae(CoralReefLocation.H, CoralLevel.L4), algae(CoralReefLocation.I)));
        autoChooser.addOption("Test", List.of(
                singleCoral(CoralReefLocation.K, CoralLevel.L4)));
    }

    public record AutoStep(List<SelectButtonInfo<?>> buttons, List<ButtonInfo> otherButtons, BooleanSupplier exit,
            Runnable onEnter) {
    }

    public AutoStep scoreCoral(CoralReefLocation location, CoralLevel level, IntakeStation station) {
        List<SelectButtonInfo<?>> buttons = List.of(coralLocationBtn(location), coralLevelBtn(level),
                intakeStationBtn(station));
        AtomicBoolean hadCoral = new AtomicBoolean(true);

        return new AutoStep(buttons, List.of(), () -> {
            boolean hasCoral = hasCoralDebouncer.calculate(endEffector.hasCoral());
            if (hasCoral && !hadCoral.get()) {
                return true;
            }
            hadCoral.set(hasCoral);
            return false;
        }, () -> {
        });
    }

    public AutoStep singleCoral(CoralReefLocation location, CoralLevel level) {
        List<SelectButtonInfo<?>> buttons = List.of(coralLocationBtn(location), coralLevelBtn(level));
        // List<ButtonInfo> otherButtons = List.of(board.algaeModeToggle);

        return new AutoStep(buttons, List.of(), () -> false, () -> {
        });
    }

    public AutoStep singleCoralIntakeAlgae(CoralReefLocation location, CoralLevel level) {
        List<SelectButtonInfo<?>> buttons = List.of(coralLocationBtn(location), coralLevelBtn(level));
        List<ButtonInfo> otherButtons = List.of(board.algaeModeToggle, board.net);

        Timer algaeTimer = new Timer();
        return new AutoStep(buttons, otherButtons,
                () -> endEffector.hasAlgae() || algaeTimer.get() > algaeMaxIntakeTime.get(), () -> {
                    algaeTimer.reset();
                    algaeTimer.start();
                });
    }

    public AutoStep algae(CoralReefLocation location) {
        List<SelectButtonInfo<?>> buttons = List.of(coralLocationBtn(location));
        List<ButtonInfo> otherButtons = List.of(board.algaeModeToggle, board.net);

        return new AutoStep(buttons, otherButtons, () -> endEffector.hasAlgae(), () -> {
        });
    }

    private SelectButtonInfo<CoralReefLocation> coralLocationBtn(CoralReefLocation location) {
        return board.reefButtons.get(location.ordinal());
    }

    private SelectButtonInfo<CoralLevel> coralLevelBtn(CoralLevel level) {
        return board.levels.get(level.ordinal());
    }

    private SelectButtonInfo<IntakeStation> intakeStationBtn(IntakeStation station) {
        return board.intakeStationButtons.get(station.ordinal());
    }
}
