package frc.robot;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.util.AutoTargetUtils.IntakeStations.IntakeStation;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.AutoTargetUtils.Reef.CoralReefLocation;
import frc.robot.util.bboard.ButtonBoard;
import frc.robot.util.bboard.ButtonBoard.SelectButtonInfo;

public class Autos {
    private final ButtonBoard board;
    private final EndEffector endEffector;

    public Autos(ButtonBoard board, EndEffector endEffector) {
        this.board = board;
        this.endEffector = endEffector;
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
