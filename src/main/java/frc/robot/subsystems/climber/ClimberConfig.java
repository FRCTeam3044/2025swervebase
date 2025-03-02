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
        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).limitSwitch
                .reverseLimitSwitchEnabled(true);
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(followerCanId, true);
        followerConfig.signals.primaryEncoderPositionPeriodMs(500).primaryEncoderVelocityPeriodMs(500);
        leaderConfig.signals.primaryEncoderPositionPeriodMs(500).primaryEncoderVelocityPeriodMs(500);
        softLimitConfig.forwardSoftLimit(forwardSoftLimit).forwardSoftLimitEnabled(true);

        leaderConfig.apply(softLimitConfig);
    }
}
