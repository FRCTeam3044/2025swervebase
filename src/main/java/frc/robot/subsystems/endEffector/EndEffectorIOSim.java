package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class EndEffectorIOSim implements EndEffectorIO {
    private XboxController controller;

    public EndEffectorIOSim(XboxController controller) {
        this.controller = controller;
        SmartDashboard.putBoolean("HasCoral", false);
        SmartDashboard.putBoolean("HasAlgae", false);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        if (controller.getAButtonPressed()) {
            SmartDashboard.putBoolean("HasCoral", !SmartDashboard.getBoolean("HasCoral", false));
        }
        if (controller.getBButtonPressed()) {
            SmartDashboard.putBoolean("HasAlgae", !SmartDashboard.getBoolean("HasAlgae", false));
        }
        inputs.hasCoral = SmartDashboard.getBoolean("HasCoral", false);
        inputs.hasAlgae = SmartDashboard.getBoolean("HasAlgae", false);
    }

    @Override
    public void setSpeed(double speed) {
        if (speed < 0 && SmartDashboard.getBoolean("HasCoral", false)) {
            RobotContainer.getInstance().shootCoral();
            SmartDashboard.putBoolean("HasCoral", false);
        }
    }
}
