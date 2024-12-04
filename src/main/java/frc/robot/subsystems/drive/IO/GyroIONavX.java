// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.Degrees;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.SerialPort.Port;

import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIONavX implements GyroIO {
    private final AHRS navx;
    private final Queue<Angle> yawPositionInput;

    public GyroIONavX() {
        this(Port.kMXP);
    }

    public GyroIONavX(Port port) {
        navx = new AHRS(port);
        navx.zeroYaw();

        yawPositionInput = OdometryThread.registerInput(() -> Angle.ofBaseUnits(navx.getYaw(), Degrees));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.yawVelocityRadPerSec = navx.getVelocityZ();

        inputs.odometryYawPositions =
                yawPositionInput.stream().map(Rotation2d::new).toArray(Rotation2d[]::new);

        yawPositionInput.clear();

        if (inputs.odometryYawPositions.length > 0)
            inputs.yawPosition = inputs.odometryYawPositions[inputs.odometryYawPositions.length - 1];
    }
}
