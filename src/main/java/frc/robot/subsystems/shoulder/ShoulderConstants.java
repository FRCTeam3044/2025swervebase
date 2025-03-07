package frc.robot.subsystems.shoulder;

public class ShoulderConstants {
    public static final int leaderCanId = 29;
    public static final int followerCanId = 28;
    public static final double shoulderMotorReduction = 45.0;
    public static final int currentLimit = 40;

    public static final double forwardSoftLimit = 4.92;
    public static final double reverseSoftLimit = 0.11;

    public static double kDt = 0.02;
    public static double kMaxVelocity = 6;
    public static double kMaxAcceleration = 5;
    public static double kP = 0;
    public static double kI = 0.0;
    public static double kD = 0;
    public static double kS = 0;
    public static double kG = 1.3055;
    public static double kV = 0;
    public static double kOffsetToHoriz = -1.9143;
}
