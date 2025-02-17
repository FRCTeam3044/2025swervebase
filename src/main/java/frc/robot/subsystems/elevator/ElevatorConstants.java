package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int leaderCanId = 21;
  public static final int followerCanId = 22;
  // Gearing is 12:1
  public static final double motorReduction = 12;
  public static final int currentLimit = 40;

  public static final double drumRadius = Units.inchesToMeters(1.432) / 2;

  public static double kDt = 0.02;
  public static double kMaxVelocity = 1.75;
  public static double kMaxAcceleration = 0.75;
  public static double kP = 1.3;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 1.1;
  public static double kG = 1.2;
  public static double kV = 1.3;

  public static final double forwardSoftLimit = 0.0;
  public static final double reverseSoftLimit = 0.0;
}