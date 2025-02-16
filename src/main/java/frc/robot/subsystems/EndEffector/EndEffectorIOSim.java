package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorIOSim implements EndEffectorIO {
    private XboxController controller;

    public EndEffectorIOSim(XboxController controller) {
        this.controller = controller;
        SmartDashboard.putBoolean("HasCoral", false);
        SmartDashboard.putBoolean("HasAlgae", false);
    }

    @Override
    public boolean hasCoral() {
        if(controller.getAButtonPressed()) {
            SmartDashboard.putBoolean("HasCoral", !SmartDashboard.getBoolean("HasCoral", false));
        }
        return SmartDashboard.getBoolean("HasCoral", false);
    }

    @Override
    public boolean hasAlgae() {
        if(controller.getBButtonPressed()) {
            SmartDashboard.putBoolean("HasAlgae", !SmartDashboard.getBoolean("HasAlgae", false));
        }
        return SmartDashboard.getBoolean("HasAlgae", false);
    }
}
