package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConfigs {
    // Leader
    public static SparkMaxConfig leftConfig = new SparkMaxConfig();
    // Follower
    public static SparkMaxConfig rightConfig = new SparkMaxConfig();

    public static SoftLimitConfig leftSoftConfig = new SoftLimitConfig();
    public static SoftLimitConfig rightSoftConfig = new SoftLimitConfig();

    static {
        // TODO: Soft Limit conversion factor
        leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);
        rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(leftCanId);

        leftSoftConfig.forwardSoftLimit(0.0);
        leftSoftConfig.reverseSoftLimit(0.0);
        rightSoftConfig.forwardSoftLimit(0.0);
        rightSoftConfig.reverseSoftLimit(0.0);
        
        leftConfig.apply(leftSoftConfig);
        rightConfig.apply(rightSoftConfig);
    }
}
