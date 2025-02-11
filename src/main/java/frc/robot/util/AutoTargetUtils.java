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
                    POIData.create(0, 0, 0, 0), // A
                    POIData.create(0, 0, 0, 0), // B
                    POIData.create(0, 0, 0, 0), // C
                    POIData.create(0, 0, 0, 0), // D
                    POIData.create(0, 0, 0, 0), // E
                    POIData.create(0, 0, 0, 0), // F
                    POIData.create(0, 0, 0, 0), // G
                    POIData.create(0, 0, 0, 0), // H
                    POIData.create(0, 0, 0, 0), // I
                    POIData.create(0, 0, 0, 0), // J
                    POIData.create(0, 0, 0, 0), // K
                    POIData.create(0, 0, 0, 0) // L
            };

            public POIData data() {
                return corals[ordinal()];
            }

            public Pose2d pose() {
                return AllianceUtil.getPoseForAlliance(corals[ordinal()].pos.asPose2d());
            }
        }

        public static final ConfigurableParameter<Double> coralL1Distance = new ConfigurableParameter<Double>(1.0,
                "Coral L1 scoring dist");
        public static final ConfigurableParameter<Double> coralL2Distance = new ConfigurableParameter<Double>(1.0,
                "Coral L2 scoring dist");
        public static final ConfigurableParameter<Double> coralL3Distance = new ConfigurableParameter<Double>(1.0,
                "Coral L3 scoring dist");
        public static final ConfigurableParameter<Double> coralL4Distance = new ConfigurableParameter<Double>(1.0,
                "Coral L4 scoring dist");
        public static final ConfigurableParameter<Boolean> flipped = new ConfigurableParameter<Boolean>(false,
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
                    POIData.create(0, 0, 0, 0), // AB
                    POIData.create(0, 0, 0, 0), // CD
                    POIData.create(0, 0, 0, 0), // EF
                    POIData.create(0, 0, 0, 0), // GH
                    POIData.create(0, 0, 0, 0), // IJ
                    POIData.create(0, 0, 0, 0), // KL
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
                    POIData.create(0, 0, 0, 0), // LeftOne
                    POIData.create(0, 0, 0, 0), // LeftTwo
                    POIData.create(0, 0, 0, 0), // LeftThree
                    POIData.create(0, 0, 0, 0), // RightOne
                    POIData.create(0, 0, 0, 0), // RightTwo
                    POIData.create(0, 0, 0, 0), // RightThree
            };

            public POIData data() {
                return stations[ordinal()];
            }

            public Pose2d pose() {
                return AllianceUtil.getPoseForAlliance(stations[ordinal()].pos.asPose2d());
            }
        }

        public static ConfigurableParameter<Double> intakeStationDistance = new ConfigurableParameter<Double>(1.0,
                "Intake station dist");

        public static Pose2d intakeStation(IntakeStation station) {
            return station.data().poseWithRot(intakeStationDistance.get(), new Rotation2d());
        }
    }

    public static Pose2d processor = new Pose2d(0, 0, new Rotation2d());

    public static Pose2d processor() {
        return AllianceUtil.getPoseForAlliance(processor);
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

    // Current Selections

}
