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
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(leaderCanId);
        followerConfig.signals.primaryEncoderPositionPeriodMs(500).primaryEncoderVelocityPeriodMs(500);

        leaderConfig.alternateEncoder.countsPerRevolution(8192).setSparkMaxDataPortConfig();
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        softLimitConfig.forwardSoftLimit(forwardSoftLimit);
        softLimitConfig.reverseSoftLimit(reverseSoftLimit);

        leaderConfig.apply(softLimitConfig);
    }
}
