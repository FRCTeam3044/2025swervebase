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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class DriveConstants {
        public static final ConfigurableParameter<Double> maxSpeedMetersPerSec = new ConfigurableParameter<Double>(4.8,
                        "Drive Speed (m/s)");
        public static final double odometryFrequency = 100.0; // Hz
        public static final double trackWidth = Units.inchesToMeters(26);
        public static final double wheelBase = Units.inchesToMeters(26);
        public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        // Zeroed rotation values for each module, see setup instructions
        public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2);
        public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
        public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
        public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

        // Device CAN IDs

        public static final int frontLeftDriveCanId = 17;
        public static final int backLeftDriveCanId = 11;
        public static final int frontRightDriveCanId = 15;
        public static final int backRightDriveCanId = 13;

        public static final int frontLeftTurnCanId = 18;
        public static final int backLeftTurnCanId = 12;
        public static final int frontRightTurnCanId = 16;
        public static final int backRightTurnCanId = 14;

        // Drive motor configuration
        public static final int driveMotorCurrentLimit = 40;
        public static final double wheelRadiusMeters = Units.inchesToMeters(1.408);
        public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion
                                                                                        // teeth
        // and 22 spur teeth
        public static final DCMotor driveGearbox = DCMotor.getNEO(1);

        // Drive encoder configuration
        public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
        // Wheel Radians
        public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
                                                                                                            // ->
        // Wheel Rad/Sec

        // Drive PID configuration
        public static final double driveKp = 0.0;
        public static final double driveKd = 0.0;
        public static final double driveKs = 0.21965;
        public static final double driveKv = 0.09869;
        public static final double driveSimP = 0.05;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.0;
        public static final double driveSimKv = 0.0789;

        // Turn motor configuration
        public static final boolean turnInverted = false;
        public static final int turnMotorCurrentLimit = 20;
        public static final double turnMotorReduction = 9424.0 / 203.0;
        public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

        // Turn encoder configuration
        public static final boolean turnEncoderInverted = true;
        public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
        public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

        // Turn PID configuration
        public static final double turnKp = 2.0;
        public static final double turnKd = 0.0;
        public static final double turnSimP = 8.0;
        public static final double turnSimD = 0.0;
        public static final double turnPIDMinInput = 0; // Radians
        public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

        // MapleSim configuration
        public static final Mass robotMassKg = Kilogram.of(74.088);
        public static final double wheelCOF = 1.43;
        public static final Distance bumperSize = Inches.of(36.125);
        // Make crab bot look right in ascope
        public static final Distance mapleBumperSize = Constants.currentMode == Mode.SIM
                        ? Inches.of(36.25)
                        : bumperSize;

        public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
                        .withBumperSize(mapleBumperSize, mapleBumperSize)
                        .withCustomModuleTranslations(moduleTranslations)
                        .withRobotMass(robotMassKg)
                        .withGyro(COTS.ofNav2X())
                        .withTrackLengthTrackWidth(Meters.of(wheelBase), Meters.of(trackWidth))
                        .withSwerveModule(COTS.ofMAXSwerve(driveGearbox, turnGearbox, wheelCOF, 2));

        public static final Pathfinder pathfinder = (new PathfinderBuilder(Field.REEFSCAPE_2025))
                        .setNormalizeCorners(false).setCornerDist(0.5)
                        .setRobotLength(mapleBumperSize.in(Meters) + 0.3)
                        .setRobotWidth(mapleBumperSize.in(Meters) + 0.3).build();

        public static final PIDController xController = new ConfigurablePIDController(1, 0, 0,
                        "Pathfinding X Controller");
        public static final PIDController yController = new ConfigurablePIDController(1, 0, 0,
                        "Pathfinding Y Controller");
        public static final ProfiledPIDController angleController = new ConfigurableProfiledPIDController(
                        6.0,
                        0,
                        0,
                        // new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond.get(),
                        // kMaxAngularAccelerationRadiansPerSecondSquared.get()),
                        new TrapezoidProfile.Constraints(8, 20),
                        "Pathfinding Theta Controller");

        public static final HolonomicDriveController driveController = new HolonomicDriveController(
                        xController, yController, angleController);

        public static final PIDController xPointController = new ConfigurablePIDController(1, 0, 0,
                        "X Point Controller");
        public static final PIDController yPointController = new ConfigurablePIDController(1, 0, 0,
                        "Point Y Controller");
        public static final ProfiledPIDController anglePointController = new ConfigurableProfiledPIDController(
                        6.0,
                        0,
                        0,
                        // new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond.get(),
                        // kMaxAngularAccelerationRadiansPerSecondSquared.get()),
                        new TrapezoidProfile.Constraints(8, 20),
                        "Point Theta Controller");
        public static final HolonomicDriveController pointController = new HolonomicDriveController(
                        xPointController, yPointController, anglePointController);
}
