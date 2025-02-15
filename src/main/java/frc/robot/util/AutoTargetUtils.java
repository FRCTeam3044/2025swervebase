package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.pathfinding.structures.Vector;
import me.nabdev.pathfinding.structures.Vertex;

public class AutoTargetUtils {
    public record POIData(Vertex pos, Vector normal) {
        public static POIData create(Vertex pos1, Vertex pos2) {
            return new POIData(pos1, pos2.createVectorFrom(pos1).normalize());
        }

        public static POIData create(double pos1x, double pos1y, double pos2x, double pos2y) {
            return create(new Vertex(pos1x, pos1y), new Vertex(pos2x, pos2y));
        }

        public Pose2d poseWithRot(double distance, Rotation2d rotation) {
            Vertex robotPos = pos().moveByVector(normal().scale(distance));
            return AllianceUtil.getPoseForAlliance(new Pose2d(robotPos.x, robotPos.y, rotation));
        }

        public Pose2d poseFacing(double distance, boolean flipped) {
            Vertex robotPos = pos().moveByVector(normal().scale(distance));
            double sign = flipped ? 1 : -1;
            Rotation2d rotation = Rotation2d.fromRadians(Math.atan2(sign * normal().y, sign * normal().x));
            return AllianceUtil.getPoseForAlliance(new Pose2d(robotPos.x, robotPos.y, rotation));
        }
    }

    public static class Reef {
        public static Pose2d reef = new Pose2d(0, 0, new Rotation2d());

        public static Pose2d reef() {
            return AllianceUtil.getPoseForAlliance(reef);
        }

        // Coral
        public static enum CoralLevel {
            L1, L2, L3, L4
        }

        public static enum CoralReefLocation {
            A, B, C, D, E, F, G, H, I, J, K, L;

            public AlgaeReefLocation algae() {
                return AlgaeReefLocation.values()[ordinal() / 2];
            }

            private static POIData[] corals = {
                    POIData.create(3.69782, 4.19023, 2.76247, 4.19023), // A
                    POIData.create(3.69668, 3.86161, 2.77582, 3.86161), // B
                    POIData.create(3.94533, 3.43292, 3.54684, 2.74270), // C
                    POIData.create(4.22935, 3.26762, 3.61189, 2.19815), // D
                    POIData.create(4.72494, 3.26861, 5.25618, 2.34847), // E
                    POIData.create(5.01010, 3.43193, 5.53546, 2.52199), // F
                    POIData.create(5.25704, 3.86161, 6.17743, 3.86161), // G
                    POIData.create(5.25704, 4.19023, 6.17743, 4.19023), // H
                    POIData.create(5.00953, 4.61893, 5.57792, 5.60341), // I
                    POIData.create(4.72551, 4.78423, 5.28143, 5.74711), // J
                    POIData.create(4.22992, 4.78324, 3.79370, 5.53879), // K
                    POIData.create(3.94456, 4.61992, 3.41521, 5.53713) // L
            };

            public POIData data() {
                return corals[ordinal()];
            }

            public Pose2d pose() {
                return AllianceUtil.getPoseForAlliance(corals[ordinal()].pos.asPose2d());
            }
        }

        private static final ConfigurableParameter<Double> coralL1Distance = new ConfigurableParameter<Double>(1.0,
                "Coral L1 scoring dist");
        private static final ConfigurableParameter<Double> coralL2Distance = new ConfigurableParameter<Double>(1.0,
                "Coral L2 scoring dist");
        private static final ConfigurableParameter<Double> coralL3Distance = new ConfigurableParameter<Double>(1.0,
                "Coral L3 scoring dist");
        private static final ConfigurableParameter<Double> coralL4Distance = new ConfigurableParameter<Double>(1.0,
                "Coral L4 scoring dist");
        private static final ConfigurableParameter<Boolean> flipped = new ConfigurableParameter<Boolean>(false,
                "Coral Flipped");

        public static double coralDistance(CoralLevel level) {
            switch (level) {
                case L1:
                    return coralL1Distance.get();
                case L2:
                    return coralL2Distance.get();
                case L3:
                    return coralL3Distance.get();
                case L4:
                    return coralL4Distance.get();
                default:
                    return 0;
            }
        }

        public static Pose2d coral(CoralReefLocation location, CoralLevel level) {
            return location.data().poseFacing(coralDistance(level), flipped.get());
        }

        public static enum AlgaeReefLocation {
            AB(true), CD(false), EF(true), GH(false), IJ(true), KL(false);

            // False: Between L2-L3
            // True: Between L3-L4
            private final boolean upperBranch;

            AlgaeReefLocation(boolean upperBranch) {
                this.upperBranch = upperBranch;
            }

            public static POIData[] algaes = {
                POIData.create(3.69725191, 4.02592267, 2.31406077, 4.02592267), // AB
                POIData.create(4.08734157, 3.35026829, 3.50063442, 2.33406171), // CD
                POIData.create(4.86752025, 3.35026865, 5.36061874, 2.49619702), // EF
                POIData.create(5.25760928, 4.0259234, 6.70907787, 4.0259234), // GH
                POIData.create(4.86751962, 4.70157778, 5.68693887, 6.12085356), // IJ
                POIData.create(4.08734093, 4.70157742, 3.47964389, 5.75413957), // KL
            };

            public POIData data() {
                return algaes[ordinal()];
            }

            public Pose2d pose() {
                return AllianceUtil.getPoseForAlliance(algaes[ordinal()].pos.asPose2d());
            }

            public boolean upperBranch() {
                return upperBranch;
            }
        }

        public static ConfigurableParameter<Double> algaeLowDistance = new ConfigurableParameter<Double>(1.0,
                "Algae low scoring dist");
        public static ConfigurableParameter<Double> algaeHighDistance = new ConfigurableParameter<Double>(1.0,
                "Algae high scoring dist");
        public static ConfigurableParameter<Boolean> algaeFlipped = new ConfigurableParameter<Boolean>(false,
                "Algae Flipped");

        public static Pose2d algae(AlgaeReefLocation location) {
            return location.data().poseFacing(location.upperBranch() ? algaeHighDistance.get() : algaeLowDistance.get(),
                    algaeFlipped.get());
        }
    }

    public static class IntakeStations {
        public static enum IntakeStation {
            LeftOne, LeftTwo, LeftThree, RightOne, RightTwo, RightThree;

            private static POIData[] stations = {
                    POIData.create(1.17152, 7.64678, 1.35439, 7.39497), // LeftOne
                    POIData.create(0.84268, 7.40797, 1.00537, 7.18395), // LeftTwo
                    POIData.create(0.51385, 7.16916, 0.68402, 6.93484), // LeftThree
                    POIData.create(1.17152, 0.40502, 1.35439, 0.65683), // RightOne
                    POIData.create(0.84268, 0.64383, 1.00537, 0.86785), // RightTwo
                    POIData.create(0.51385, 0.88264, 0.68402, 1.11696), // RightThree
            };

            public POIData data() {
                return stations[ordinal()];
            }

            public Pose2d pose() {
                return AllianceUtil.getPoseForAlliance(stations[ordinal()].pos.asPose2d());
            }

        }

        private static ConfigurableParameter<Double> intakeStationDistance = new ConfigurableParameter<Double>(1.0,
                "Intake station dist");

        private static ConfigurableParameter<Boolean> intakeStationFlipped = new ConfigurableParameter<Boolean>(false,
                "Intake station flipped");

        public static Pose2d intakeStation(IntakeStation station) {
            return station.data().poseFacing(intakeStationDistance.get(), intakeStationFlipped.get());
        }
    }

    private static POIData processor = POIData.create(0, 0, 0, 0);
    private static ConfigurableParameter<Double> processorDistance = new ConfigurableParameter<Double>(1.0,
            "Processor dist");
    private static ConfigurableParameter<Boolean> processorFlipped = new ConfigurableParameter<Boolean>(false,
            "Processor Flipped");

    public static Pose2d processor() {
        return processor.poseFacing(processorDistance.get(), processorFlipped.get());
    }

    public static DoubleSupplier robotDistToPose(Drive drive, Supplier<Pose2d> pose) {
        return () -> {
            Translation2d robot = drive.getPose().getTranslation();
            return pose.get().getTranslation().getDistance(robot);
        };
    }

    public static double robotDistToPose(Drive drive, Pose2d pose) {
        Translation2d robot = drive.getPose().getTranslation();
        return pose.getTranslation().getDistance(robot);
    }
}
