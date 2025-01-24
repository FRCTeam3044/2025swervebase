package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

public class AutoTargetUtils {
    // Hardcode positions
    public static Pose2d blueReef = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefA = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefB = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefC = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefD = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefE = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefF = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefG = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefH = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefI = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefJ = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefK = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueReefL = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueRightStation1 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueRightStation1Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueRightStation2 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueRightStation2Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueRightStation3 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueRightStation3Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueLeftStation1 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueLeftStation1Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueLeftStation2 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueLeftStation2Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueLeftStation3 = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueLeftStation3Ref = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d blueProcessor = new Pose2d(0, 0, new Rotation2d());

    public static Pose2d reef(){
        return AllianceUtil.getPoseForAlliance(blueReef);
    }

    public static Pose2d reefA(){
        return AllianceUtil.getPoseForAlliance(blueReefA);
    }

    public static Pose2d reefB(){
        return AllianceUtil.getPoseForAlliance(blueReefB);
    }

    public static Pose2d reefC(){
        return AllianceUtil.getPoseForAlliance(blueReefC);
    }

    public static Pose2d reefD(){
        return AllianceUtil.getPoseForAlliance(blueReefD);
    }

    public static Pose2d reefE(){
        return AllianceUtil.getPoseForAlliance(blueReefE);
    }

    public static Pose2d reefF(){
        return AllianceUtil.getPoseForAlliance(blueReefF);
    }

    public static Pose2d reefG(){
        return AllianceUtil.getPoseForAlliance(blueReefG);
    }

    public static Pose2d reefH(){
        return AllianceUtil.getPoseForAlliance(blueReefH);
    }

    public static Pose2d reefI(){
        return AllianceUtil.getPoseForAlliance(blueReefI);
    }

    public static Pose2d reefJ(){
        return AllianceUtil.getPoseForAlliance(blueReefJ);
    }

    public static Pose2d reefK(){
        return AllianceUtil.getPoseForAlliance(blueReefK);
    }

    public static Pose2d reefL(){
        return AllianceUtil.getPoseForAlliance(blueReefL);
    }

    public static Pose2d blueRightStation1(){
        return AllianceUtil.getPoseForAlliance(blueRightStation1);
    }
    
    public static Pose2d blueRightStation1Ref(){
        return AllianceUtil.getPoseForAlliance(blueRightStation1Ref);
    }

    public static Pose2d blueRightStation2(){
        return AllianceUtil.getPoseForAlliance(blueRightStation2);
    }

    public static Pose2d blueRightStation2Ref(){
        return AllianceUtil.getPoseForAlliance(blueRightStation2Ref);
    }

    public static Pose2d blueRightStation3(){
        return AllianceUtil.getPoseForAlliance(blueRightStation3);
    }

    public static Pose2d blueRightStation3Ref(){
        return AllianceUtil.getPoseForAlliance(blueRightStation3Ref);
    }

    public static Pose2d blueLeftStation1(){
        return AllianceUtil.getPoseForAlliance(blueLeftStation1);
    }

    public static Pose2d blueLeftStation1Ref(){
        return AllianceUtil.getPoseForAlliance(blueLeftStation1Ref);
    }

    public static Pose2d blueLeftStation2(){
        return AllianceUtil.getPoseForAlliance(blueLeftStation2);
    }

    public static Pose2d blueLeftStation2Ref(){
        return AllianceUtil.getPoseForAlliance(blueLeftStation2Ref);
    }

    public static Pose2d blueLeftStation3(){
        return AllianceUtil.getPoseForAlliance(blueLeftStation3);
    }

    public static Pose2d blueLeftStation3Ref(){
        return AllianceUtil.getPoseForAlliance(blueLeftStation3Ref);
    }

    public static Pose2d blueProcessor(){
        return AllianceUtil.getPoseForAlliance(blueProcessor);
    }

    public static DoubleSupplier robotDistToReef(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::reef);
    }

    public static DoubleSupplier robotDistToLeftStation1(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::blueLeftStation1);
    }

    public static DoubleSupplier robotDistToLeftStation2(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::blueLeftStation2);
    }

    public static DoubleSupplier robotDistToLeftStation3(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::blueLeftStation3);
    }

    public static DoubleSupplier robotDistToRightStation1(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::blueRightStation1);
    }

    public static DoubleSupplier robotDistToRightStation2(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::blueRightStation2);
    }

    public static DoubleSupplier robotDistToRightStation3(Drive drive){
        return robotDistToPose(drive, AutoTargetUtils::blueRightStation3);
    }

    public static DoubleSupplier robotDistToPose(Drive drive, Supplier<Pose2d> pose){
        return () -> {
            Translation2d robot = drive.getPose().getTranslation();
            return pose.get().getTranslation().getDistance(robot);
        };
    }
}
