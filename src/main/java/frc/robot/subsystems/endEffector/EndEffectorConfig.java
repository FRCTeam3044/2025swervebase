package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.spark.config.SparkMaxConfig;

public class EndEffectorConfig {
    public static SparkMaxConfig motorConfig = new SparkMaxConfig();

    static {
        motorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(currentLimit).inverted(true);
    }
}
