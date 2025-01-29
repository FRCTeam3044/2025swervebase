package frc.robot.subsystems.EndEffector;

import me.nabdev.oxconfig.ConfigurableParameter;

public class EndEffectorConstants {
    public static final int coralCanId = 31;
    public static final double coralMotorReduction = 1.0;
    public static final double algaeMotorReduction = 1.0;
    public static final int currentLimit = 40;
    public static final ConfigurableParameter<Double> intakeSpeed = new ConfigurableParameter<>(1.0, "Intake speed");
}
