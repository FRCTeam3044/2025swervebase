package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static frc.robot.subsystems.climber.ClimberConstants.*;

public class ClimberConfig {
    // Leader
    public static SparkMaxConfig rightMotorClimber = new SparkMaxConfig();
    // Follower
    public static SparkMaxConfig leftMotorClimber = new SparkMaxConfig();

    static {
        leftMotorClimber.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).inverted(true);
        rightMotorClimber.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).follow(leftMotorCanId);
    }
}
