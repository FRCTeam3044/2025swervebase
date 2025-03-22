// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionConstants {
    // AprilTag layout
    // public static AprilTagFieldLayout aprilTagLayout =
    // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static AprilTagFieldLayout aprilTagLayout;

    static {
        try {
            // aprilTagLayout =
            // AprilTagFieldLayout.loadFromResource(Filesystem.getDeployDirectory().toPath()
            // + "/practice_field.json");
            aprilTagLayout = AprilTagFieldLayout.loadFromResource("/Users/alex/Downloads/field_calibration.json");
            SmartDashboard.putBoolean("Using custom field layout", true);
        } catch (IOException e) {
            e.printStackTrace();
            aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
            SmartDashboard.putBoolean("Using custom field layout", false);
        }
    }

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "elevator";
    public static String camera1Name = "swerve_port";
    public static String camera2Name = "swerve_star";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(0.004028, 0.325, 1.0033,
            new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(187)));

    public static Transform3d robotToCamera1 = new Transform3d(0.254, 0.244475, 0.14605,
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-24.5)));

    public static Transform3d robotToCamera2 = new Transform3d(0.254, -0.244475, 0.14605,
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(24.5)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.2;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.1; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            0.8, // Camera 0
            1.2, // Camera 1
            1.2 // Camera 2
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}