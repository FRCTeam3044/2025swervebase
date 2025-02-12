package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorIOSim implements EndEffectorIO {

    public EndEffectorIOSim() {
        SmartDashboard.putBoolean("HasCoral", false);
        SmartDashboard.putBoolean("HasAlgae", false);
    }

    @Override
    public boolean hasCoral() {
        return SmartDashboard.getBoolean("HasCoral", false);
    }

    @Override
    public boolean hasAlgae() {
        return SmartDashboard.getBoolean("HasAlgae", false);
    }
}
