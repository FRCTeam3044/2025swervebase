package frc.robot;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.util.AutoTargetUtils.IntakeStations.IntakeStation;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.AutoTargetUtils.Reef.CoralReefLocation;
import frc.robot.util.bboard.ButtonBoard;
import frc.robot.util.bboard.ButtonBoard.SelectButtonInfo;

public class Autos {
    private final ButtonBoard board;
    private final EndEffector endEffector;
    public final LoggedDashboardChooser<List<AutoStep>> autoChooser;

    public Autos(ButtonBoard board, EndEffector endEffector) {
        this.board = board;
        this.endEffector = endEffector;

        autoChooser = new LoggedDashboardChooser<>("Auto Routine");
        autoChooser.addDefaultOption("None", List.of());
        autoChooser.addOption("Right L4", List.of(
                scoreCoral(CoralReefLocation.E, CoralLevel.L4, IntakeStation.RightTwo),
                scoreCoral(CoralReefLocation.D, CoralLevel.L4, IntakeStation.RightTwo),
                scoreCoral(CoralReefLocation.C, CoralLevel.L4, IntakeStation.RightTwo)));
        autoChooser.addOption("Left L4", List.of(
                scoreCoral(CoralReefLocation.H, CoralLevel.L4, IntakeStation.LeftTwo),
                scoreCoral(CoralReefLocation.I, CoralLevel.L4, IntakeStation.LeftTwo),
                scoreCoral(CoralReefLocation.J, CoralLevel.L4, IntakeStation.LeftTwo)));
    }

    public record AutoStep(List<SelectButtonInfo<?>> buttons, BooleanSupplier exit) {
    }

    public AutoStep scoreCoral(CoralReefLocation location, CoralLevel level, IntakeStation station) {
        List<SelectButtonInfo<?>> buttons = List.of(coralLocationBtn(location), coralLevelBtn(level),
                intakeStationBtn(station));
        AtomicBoolean hadCoral = new AtomicBoolean(true);

        return new AutoStep(buttons, () -> {
            boolean hasCoral = endEffector.hasCoral();
            if (hasCoral && !hadCoral.get()) {
                return true;
            }
            hadCoral.set(hasCoral);
            return false;
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
