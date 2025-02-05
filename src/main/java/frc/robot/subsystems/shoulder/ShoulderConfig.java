package frc.robot.subsystems.shoulder;

import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static frc.robot.subsystems.shoulder.ShoulderConstants.*;

public class ShoulderConfig {
    // Leader
    public static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    // Follower
    public static SparkMaxConfig followerConfig = new SparkMaxConfig();

    public static SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    static {
        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(leaderCanId);

        softLimitConfig.forwardSoftLimit(forwardSoftLimit);
        softLimitConfig.reverseSoftLimit(reverseSoftLimit);

        leaderConfig.apply(softLimitConfig);
    }
}
