package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

public class AutoTargetUtils {
    // Hardcode positions
    public static Pose2d reef = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefA = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefB = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefC = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefD = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefE = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefF = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefG = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefH = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefI = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefJ = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefK = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d reefL = new Pose2d(0, 0, new Rotation2d());
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
    public static Pose2d rrocessor = new Pose2d(0, 0, new Rotation2d());

    public static Pose2d reef(){
        return AllianceUtil.getPoseForAlliance(reef);
    }

    public static Pose2d reefA(){
        return AllianceUtil.getPoseForAlliance(reefA);
    }

    public static Pose2d reefB(){
        return AllianceUtil.getPoseForAlliance(reefB);
    }

    public static Pose2d reefC(){
        return AllianceUtil.getPoseForAlliance(reefC);
    }

    public static Pose2d reefD(){
        return AllianceUtil.getPoseForAlliance(reefD);
    }

    public static Pose2d reefE(){
        return AllianceUtil.getPoseForAlliance(reefE);
    }

    public static Pose2d reefF(){
        return AllianceUtil.getPoseForAlliance(reefF);
    }

    public static Pose2d reefG(){
        return AllianceUtil.getPoseForAlliance(reefG);
    }

    public static Pose2d reefH(){
        return AllianceUtil.getPoseForAlliance(reefH);
    }

    public static Pose2d reefI(){
        return AllianceUtil.getPoseForAlliance(reefI);
    }

    public static Pose2d reefJ(){
        return AllianceUtil.getPoseForAlliance(reefJ);
    }

    public static Pose2d reefK(){
        return AllianceUtil.getPoseForAlliance(reefK);
    }

    public static Pose2d reefL(){
        return AllianceUtil.getPoseForAlliance(reefL);
    }

    public static Pose2d rightStation1(){
        return AllianceUtil.getPoseForAlliance(rightStation1);
    }
    
    public static Pose2d rightStation1Ref(){
        return AllianceUtil.getPoseForAlliance(rightStation1Ref);
    }

    public static Pose2d rightStation2(){
        return AllianceUtil.getPoseForAlliance(rightStation2);
    }

    public static Pose2d rightStation2Ref(){
        return AllianceUtil.getPoseForAlliance(rightStation2Ref);
    }

    public static Pose2d rightStation3(){
        return AllianceUtil.getPoseForAlliance(rightStation3);
    }

    public static Pose2d rightStation3Ref(){
        return AllianceUtil.getPoseForAlliance(rightStation3Ref);
    }

    public static Pose2d leftStation1(){
        return AllianceUtil.getPoseForAlliance(reftStation1);
    }

    public static Pose2d leftStation1Ref(){
        return AllianceUtil.getPoseForAlliance(reftStation1Ref);
    }

    public static Pose2d leftStation2(){
        return AllianceUtil.getPoseForAlliance(reftStation2);
    }

    public static Pose2d leftStation2Ref(){
        return AllianceUtil.getPoseForAlliance(reftStation2Ref);
    }

    public static Pose2d leftStation3(){
        return AllianceUtil.getPoseForAlliance(reftStation3);
    }

    public static Pose2d leftStation3Ref(){
        return AllianceUtil.getPoseForAlliance(reftStation3Ref);
    }

    public static Pose2d processor(){
        return AllianceUtil.getPoseForAlliance(rrocessor);
    }

    public static DoubleSupplier robotDistToReef(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::reef);
    }

    public static DoubleSupplier robotDistToLeftStation1(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::leftStation1);
    }

    public static DoubleSupplier robotDistToLeftStation2(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::leftStation2);
    }

    public static DoubleSupplier robotDistToLeftStation3(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::leftStation3);
    }

    public static DoubleSupplier robotDistToRightStation1(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::rightStation1);
    }

    public static DoubleSupplier robotDistToRightStation2(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::rightStation2);
    }

    public static DoubleSupplier robotDistToRightStation3(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::rightStation3);
    }

    public static DoubleSupplier robotDistToPose(Drive drive, Supplier<Pose2d> pose){
        return () -> {
            Translation2d robot = drive.getPose().getTranslation();
            return pose.get().getTranslation().getDistance(robot);
        };
    }
}
