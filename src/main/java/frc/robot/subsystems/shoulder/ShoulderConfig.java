package frc.robot.subsystems.shoulder;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import static frc.robot.subsystems.shoulder.ShoulderConstants.*;

public class ShoulderConfig {
    // Leader
    public static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    // Follower
    public static SparkMaxConfig followerConfig = new SparkMaxConfig();

    public static SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    public static AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();

    static {
        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(leaderCanId, true);
        followerConfig.signals.primaryEncoderPositionPeriodMs(500).primaryEncoderVelocityPeriodMs(500);
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).positionWrappingEnabled(false);

        softLimitConfig.forwardSoftLimit(forwardSoftLimit).forwardSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimit(reverseSoftLimit).reverseSoftLimitEnabled(true);

        absoluteEncoderConfig.positionConversionFactor(2 * Math.PI).inverted(true);

        leaderConfig.apply(absoluteEncoderConfig);
        leaderConfig.apply(softLimitConfig);
    }
}
