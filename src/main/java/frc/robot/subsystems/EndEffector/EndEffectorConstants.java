package frc.robot.subsystems.EndEffector;

import me.nabdev.oxconfig.ConfigurableParameter;

public class EndEffectorConstants {
    public static final int canId = 31;
    public static final int proximityChannel = 0;
    public static final double coralMotorReduction = 1.0;
    public static final int currentLimit = 40;
    public static final ConfigurableParameter<Double> intakeSpeed = new ConfigurableParameter<>(1.0, "Intake speed");
    public static final ConfigurableParameter<Double> coralProximityDistance = new ConfigurableParameter<Double>(15.0,
            "Coral proximity distance");
    public static final ConfigurableParameter<Double> currentThreshold = new ConfigurableParameter<Double>(1.0,
            "Current threshold");
}
