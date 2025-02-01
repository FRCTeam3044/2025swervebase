package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConfigs {
    // Leader
    public static SparkMaxConfig leftConfig = new SparkMaxConfig();
    // Follower
    public static SparkMaxConfig rightConfig = new SparkMaxConfig();

    static {
        // TODO:Soft Limit
        leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);
        rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(leftCanId);
    }
}
