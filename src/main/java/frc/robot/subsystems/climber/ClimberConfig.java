package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static frc.robot.subsystems.climber.ClimberConstants.*;

public class ClimberConfig {
    // leaderConfig
    public static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    // followerConfig
    public static SparkMaxConfig followerConfig = new SparkMaxConfig();

    public static SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    static {
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);
        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(followerCanId);

        softLimitConfig.forwardSoftLimit(forwardSoftLimit);
        softLimitConfig.reverseSoftLimit(reverseSoftLimit);

        leaderConfig.apply(softLimitConfig);
    }
}
