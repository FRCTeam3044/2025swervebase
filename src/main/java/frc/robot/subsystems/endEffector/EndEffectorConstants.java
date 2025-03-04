package frc.robot.subsystems.endEffector;

import me.nabdev.oxconfig.ConfigurableParameter;

public class EndEffectorConstants {
        public static final int canId = 31;
        public static final int proximityChannel = 0;
        public static final double coralMotorReduction = 1.0;
        public static final int currentLimit = 40;
        public static final ConfigurableParameter<Double> algaeInSpeed = new ConfigurableParameter<>(1.0,
                        "Intake algae in speed");
        public static final ConfigurableParameter<Double> algaeOutSpeed = new ConfigurableParameter<>(1.0,
                        "Intake algae out speed");
        public static final ConfigurableParameter<Double> coralInSpeed = new ConfigurableParameter<>(1.0,
                        "Intake coral in speed");
        public static final ConfigurableParameter<Double> coralOutSpeed = new ConfigurableParameter<>(1.0,
                        "Intake coral out speed");
        public static final ConfigurableParameter<Double> coralProximityDistance = new ConfigurableParameter<Double>(
                        15.0,
                        "Intake Coral proximity distance");
        public static final ConfigurableParameter<Double> currentThreshold = new ConfigurableParameter<Double>(1.0,
                        "Intake Current threshold");
}
