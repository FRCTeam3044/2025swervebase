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
            A, B, C, D, E, F, G, H, I, J, K, L
        }

        public static POIData[] corals = {
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
            return poseFacePOI(corals[location.ordinal()], coralDistance(level), flipped.get());
        }
    }

    public static Pose2d rightStation1 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d rightStation1Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d rightStation2 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d rightStation2Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d rightStation3 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d rightStation3Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reftStation1 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reftStation1Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reftStation2 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reftStation2Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reftStation3 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reftStation3Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d processor = new Pose2d(0, 0, new Rotation2d());

    public static Pose2d rightStation1() {
        return AllianceUtil.getPoseForAlliance(rightStation1);
    }

    public static Pose2d rightStation1Ref() {
        return AllianceUtil.getPoseForAlliance(rightStation1Ref);
    }

    public static Pose2d rightStation2() {
        return AllianceUtil.getPoseForAlliance(rightStation2);
    }

    public static Pose2d rightStation2Ref() {
        return AllianceUtil.getPoseForAlliance(rightStation2Ref);
    }

    public static Pose2d rightStation3() {
        return AllianceUtil.getPoseForAlliance(rightStation3);
    }

    public static Pose2d rightStation3Ref() {
        return AllianceUtil.getPoseForAlliance(rightStation3Ref);
    }

    public static Pose2d leftStation1() {
        return AllianceUtil.getPoseForAlliance(reftStation1);
    }

    public static Pose2d leftStation1Ref() {
        return AllianceUtil.getPoseForAlliance(reftStation1Ref);
    }

    public static Pose2d leftStation2() {
        return AllianceUtil.getPoseForAlliance(reftStation2);
    }

    public static Pose2d leftStation2Ref() {
        return AllianceUtil.getPoseForAlliance(reftStation2Ref);
    }

    public static Pose2d leftStation3() {
        return AllianceUtil.getPoseForAlliance(reftStation3);
    }

    public static Pose2d leftStation3Ref() {
        return AllianceUtil.getPoseForAlliance(reftStation3Ref);
    }

    public static Pose2d processor() {
        return AllianceUtil.getPoseForAlliance(processor);
    }

    public static DoubleSupplier robotDistToReef(Drive drive) {
        return robotDistToPose(drive, Reef::reef);
    }

    public static DoubleSupplier robotDistToLeftStation1(Drive drive) {
        return robotDistToPose(drive, AutoTargetUtils::leftStation1);
    }

    public static DoubleSupplier robotDistToLeftStation2(Drive drive) {
        return robotDistToPose(drive, AutoTargetUtils::leftStation2);
    }

    public static DoubleSupplier robotDistToLeftStation3(Drive drive) {
        return robotDistToPose(drive, AutoTargetUtils::leftStation3);
    }

    public static DoubleSupplier robotDistToRightStation1(Drive drive) {
        return robotDistToPose(drive, AutoTargetUtils::rightStation1);
    }

    public static DoubleSupplier robotDistToRightStation2(Drive drive) {
        return robotDistToPose(drive, AutoTargetUtils::rightStation2);
    }

    public static DoubleSupplier robotDistToRightStation3(Drive drive) {
        return robotDistToPose(drive, AutoTargetUtils::rightStation3);
    }

    public static DoubleSupplier robotDistToPose(Drive drive, Supplier<Pose2d> pose) {
        return () -> {
            Translation2d robot = drive.getPose().getTranslation();
            return pose.get().getTranslation().getDistance(robot);
        };
    }

    public static Pose2d poseFromPOI(POIData poi, double distance, Rotation2d rotation) {
        Vertex robotPos = poi.pos().moveByVector(poi.normal().scale(distance));
        return AllianceUtil.getPoseForAlliance(new Pose2d(robotPos.x, robotPos.y, rotation));
    }

    public static Pose2d poseFacePOI(POIData poi, double distance, boolean flipped) {
        Vertex robotPos = poi.pos().moveByVector(poi.normal().scale(distance));
        double sign = flipped ? 1 : -1;
        Rotation2d rotation = Rotation2d.fromRadians(Math.atan2(sign * poi.normal().y, sign * poi.normal().x));
        return AllianceUtil.getPoseForAlliance(new Pose2d(robotPos.x, robotPos.y, rotation));
    }
}
