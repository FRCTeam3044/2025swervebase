package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConfigs {
    // Leader
    public static SparkMaxConfig leaderConfig = new SparkMaxConfig();
    // Follower
    public static SparkMaxConfig followerConfig = new SparkMaxConfig();

    public static SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    static {
        // TODO: Soft Limit conversion factor
        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(leaderCanId, true);
        followerConfig.signals.primaryEncoderPositionPeriodMs(500).primaryEncoderVelocityPeriodMs(500);

        leaderConfig.alternateEncoder.countsPerRevolution(8192).inverted(true).velocityConversionFactor(1.0 / 60.0)
                .setSparkMaxDataPortConfig();
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        softLimitConfig.forwardSoftLimit(forwardSoftLimit).forwardSoftLimitEnabled(true);
        // softLimitConfig.reverseSoftLimit(reverseSoftLimit).reverseSoftLimitEnabled(true);

        leaderConfig.apply(softLimitConfig);
    }
}
