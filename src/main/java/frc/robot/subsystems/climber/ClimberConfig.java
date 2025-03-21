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
        leaderConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(currentLimit).inverted(false);
        followerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(currentLimit).follow(leaderCanId, true);
        softLimitConfig.forwardSoftLimit(forwardSoftLimit).forwardSoftLimitEnabled(true)
                .reverseSoftLimit(reverseSoftLimit).reverseSoftLimitEnabled(true);

        leaderConfig.apply(softLimitConfig);
    }
}
