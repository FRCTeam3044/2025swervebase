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

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.pathfinding.structures.Path;
import me.nabdev.pathfinding.structures.Vector;
import me.nabdev.pathfinding.structures.Vertex;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static ConfigurableParameter<Double> pathfindingMaxSpeed = new ConfigurableParameter<>(4.8,
            "Pathfinding Max Speed");
    private static ConfigurableParameter<Double> pathfindingMaxAccel = new ConfigurableParameter<>(2.0,
            "Pathfinding Max Accel");
    private static ConfigurableParameter<Double> pathfindingMaxAccelAuto = new ConfigurableParameter<>(5.0,
            "Pathfinding Max Accel (Auto)");
    private static ConfigurableParameter<Double> pathfindingRotationMaxSpeed = new ConfigurableParameter<>(Math.PI / 2,
            "Pathfinding Max Rotation Speed");
    private static ConfigurableParameter<Double> pathfindingRotationMaxSlowSpeed = new ConfigurableParameter<>(
            Math.PI / 8,
            "Pathfinding Max Rotation Slow Speed");

    public static boolean pointControllerConverged = false;
    public static boolean pointControllerLooseConverged = false;
    public static boolean pointControllerRotConverged = false;

    private DriveCommands() {
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
                },
                drive).withName("Joystick Drive");
    }

    public static Command joystickDriveRobotRel(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    drive.runVelocity(speeds);
                },
                drive).withName("Joystick Drive (Field relative)");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for
     * angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target,
     * or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {

        DriveConstants.angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Calculate angular speed
                    double omega = DriveConstants.angleController.calculate(
                            drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);

                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
                },
                drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> DriveConstants.angleController.reset(drive.getRotation().getRadians()))
                .withName("Joystick Drive At Angle");
    }

    private static Command followTrajectory(Drive drive, Trajectory traj, Supplier<Rotation2d> desiredRotation,
            DoubleSupplier joystickRot, boolean useJoystick) {

        // Ensure parameters are not null
        if (traj == null) {
            return Commands.none();
        }

        Supplier<Rotation2d> desiredRot = desiredRotation;
        if (useJoystick) {
            desiredRot = drive::getRotation;
            requireNonNullParam(joystickRot, "desiredRotationSpeed", "followTrajectory");
        } else {
            requireNonNullParam(desiredRotation, "desiredRotation", "followTrajectory");
        }
        Timer timer = new Timer();
        HolonomicDriveController m_controller = DriveConstants.driveController;

        // Change down
        final Supplier<Rotation2d> desiredRotationSupplier = desiredRot;
        return Commands.run(() -> {
            double curTime = timer.get();
            State desiredState = traj.sample(curTime);
            Logger.recordOutput("Trajectory Desired Velocity", desiredState.velocityMetersPerSecond);
            Logger.recordOutput("Trajectory Desired Pose", new Pose2d(desiredState.poseMeters.getX(),
                    desiredState.poseMeters.getY(), desiredRotationSupplier.get()));
            ChassisSpeeds targetChassisSpeeds = m_controller.calculate(drive.getPose(), desiredState,
                    desiredRotationSupplier.get());
            if (RobotContainer.getInstance().shoulder.canSpinFast()) {
                targetChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(targetChassisSpeeds.omegaRadiansPerSecond,
                        -pathfindingRotationMaxSpeed.get(), pathfindingRotationMaxSpeed.get());
            } else {
                targetChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(targetChassisSpeeds.omegaRadiansPerSecond,
                        -pathfindingRotationMaxSlowSpeed.get(), pathfindingRotationMaxSlowSpeed.get());
            }

            if (useJoystick) {
                double omega = MathUtil.applyDeadband(joystickRot.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);
                targetChassisSpeeds.omegaRadiansPerSecond = omega * drive.getMaxAngularSpeedRadPerSec();
            }
            drive.runVelocity(targetChassisSpeeds);
        }, drive).beforeStarting(() -> {
            timer.restart();
            RobotContainer.fieldSim.getObject("Path").setTrajectory(traj);
            Logger.recordOutput("Current Trajectory", traj);
        }).until(() -> timer.hasElapsed(traj.getTotalTimeSeconds()))
                .finallyDo(timer::stop).withName("Follow Trajectory");
    };

    private static Command goToPoint(Drive drive, Pose2d pose, Supplier<Rotation2d> desiredRotation,
            DoubleSupplier joystickRot, boolean useJoystick) {
        return Commands.deferredProxy(() -> followTrajectory(drive, generateTrajectory(drive, pose), desiredRotation,
                joystickRot, useJoystick)).withName("Go To Point");
    }

    public static Command goToPointJoystickRot(Drive drive, Pose2d pose, DoubleSupplier joystickRot) {
        return goToPoint(drive, pose, null, joystickRot, true).withName("Go To Point Joystick Rot");
    }

    public static Command goToPoint(Drive drive, Supplier<Pose2d> pose) {
        return Commands.deferredProxy(() -> {
            Pose2d curPose = pose.get();
            return followTrajectory(drive, generateTrajectory(drive, curPose), () -> curPose.getRotation(),
                    null, false);
        }).withName("Go To Point");
    }

    public static Command goToPoint(Drive drive, Supplier<Pose2d> pose, Supplier<Rotation2d> rotation) {
        return Commands.deferredProxy(() -> {
            Pose2d curPose = pose.get();
            return followTrajectory(drive, generateTrajectory(drive, curPose), rotation,
                    null, false);
        }).withName("Go To Point");
    }

    public static Command pointControl(Drive drive, Supplier<Pose2d> pose) {
        return Commands.startRun(() -> {
            DriveConstants.anglePointController.reset(drive.getPose().getRotation().getRadians());
        }, () -> {
            Pose2d targetPose = pose.get();
            ChassisSpeeds speeds = DriveConstants.pointController.calculate(drive.getPose(), targetPose, 0,
                    targetPose.getRotation());
            DriveConstants.pointController.setTolerance(DriveConstants.pointControllerTolerance);
            if (DriveConstants.pointController.atReference()) {
                speeds = new ChassisSpeeds(0, 0, 0);
                pointControllerConverged = true;
                pointControllerRotConverged = false;
            } else {
                pointControllerConverged = false;
            }

            if (Math.abs(targetPose.getRotation().minus(drive.getPose().getRotation())
                    .getRadians()) < 1) {
                pointControllerRotConverged = true;
            } else {
                pointControllerRotConverged = false;
            }

            DriveConstants.pointController.setTolerance(DriveConstants.pointControllerLooseTolerance);
            if (DriveConstants.pointController.atReference()) {
                pointControllerLooseConverged = true;
            } else {
                pointControllerLooseConverged = false;
            }

            drive.runVelocity(speeds);
            Logger.recordOutput("PointControllerDist",
                    drive.getPose().getTranslation().getDistance(targetPose.getTranslation()));
        }, drive).finallyDo(() -> {
            pointControllerConverged = false;
            pointControllerRotConverged = false;
        }).withName("Point Control");
    }

    public static Command pointControlFast(Drive drive, Supplier<Pose2d> pose) {
        return Commands.startRun(() -> {
            DriveConstants.anglePointControllerFast.reset(drive.getPose().getRotation().getRadians());
        }, () -> {
            Pose2d targetPose = pose.get();
            ChassisSpeeds speeds = DriveConstants.pointControllerFast.calculate(drive.getPose(), targetPose, 0,
                    targetPose.getRotation());
            DriveConstants.pointControllerFast.setTolerance(DriveConstants.pointControllerTolerance);
            if (DriveConstants.pointControllerFast.atReference()) {
                speeds = new ChassisSpeeds(0, 0, 0);
                pointControllerConverged = true;
                pointControllerRotConverged = false;
            } else {
                pointControllerConverged = false;
            }

            if (Math.abs(targetPose.getRotation().minus(drive.getPose().getRotation())
                    .getRadians()) < 1) {
                pointControllerRotConverged = true;
            } else {
                pointControllerRotConverged = false;
            }

            DriveConstants.pointControllerFast.setTolerance(DriveConstants.pointControllerLooseTolerance);
            if (DriveConstants.pointControllerFast.atReference()) {
                pointControllerLooseConverged = true;
            } else {
                pointControllerLooseConverged = false;
            }

            drive.runVelocity(speeds);
            Logger.recordOutput("PointControllerDist",
                    drive.getPose().getTranslation().getDistance(targetPose.getTranslation()));
        }, drive).finallyDo(() -> {
            pointControllerConverged = false;
            pointControllerRotConverged = false;
        }).withName("Point Control");
    }

    private static ConfigurableParameter<Double> slowMaxSpeed = new ConfigurableParameter<Double>(0.4,
            "Slow Point Controller Max");
    // private static ConfigurableParameter<Double> slowMaxSpeedAuto = new
    // ConfigurableParameter<Double>(0.5,
    // "Slow Point Controller Max Auto");
    private static ConfigurableParameter<Double> slowMaxRotSpeed = new ConfigurableParameter<Double>(0.1,
            "Slow Point Controller Max Rotation Speed");

    public static final Timer timeSincePointControl = new Timer();
    public static double initialVx = 0;
    public static double initialVy = 0;
    public static double maxVx = 0;
    public static double maxVy = 0;

    // private static ConfigurableParameter<Double> interpolateTime = new
    // ConfigurableParameter<Double>(0.5,
    // "Slow Point Controller Interpolation Time");

    public static Command pointControlSlow(Drive drive, Supplier<Pose2d> pose, BooleanSupplier slowDrive,
            BooleanSupplier slowRot) {
        return Commands.startRun(() -> {
            DriveConstants.anglePointController.reset(drive.getPose().getRotation().getRadians());
            // timeSincePointControl.reset();
            // timeSincePointControl.start();
            // ChassisSpeeds speeds = drive.getVelocity();
            // initialVx = speeds.vxMetersPerSecond;
            // initialVy = speeds.vyMetersPerSecond;
            // maxVx = initialVx;
            // maxVy = initialVy;
        }, () -> {
            Pose2d targetPose = pose.get();
            ChassisSpeeds speeds = DriveConstants.pointController.calculate(drive.getPose(), targetPose, 0,
                    targetPose.getRotation());
            DriveConstants.pointController.setTolerance(DriveConstants.pointControllerTolerance);
            if (DriveConstants.pointController.atReference()) {
                speeds = new ChassisSpeeds(0, 0, 0);
                pointControllerConverged = true;
                pointControllerRotConverged = true;
            } else {
                pointControllerConverged = false;
            }

            if (Math.abs(targetPose.getRotation().minus(drive.getPose().getRotation())
                    .getRadians()) < 1) {
                pointControllerRotConverged = true;
            } else {
                pointControllerRotConverged = false;
            }

            DriveConstants.pointController.setTolerance(DriveConstants.pointControllerLooseTolerance);
            if (DriveConstants.pointController.atReference()) {
                pointControllerLooseConverged = true;
            } else {
                pointControllerLooseConverged = false;
            }

            DoubleSupplier slowMax = () -> /* DriverStation.isAutonomous() ? slowMaxSpeedAuto.get() : */slowMaxSpeed
                    .get();
            if (slowDrive.getAsBoolean()) {
                speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -slowMax.getAsDouble(),
                        slowMax.getAsDouble());
                speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -slowMax.getAsDouble(),
                        slowMax.getAsDouble());
            }

            if (slowRot.getAsBoolean()) {
                speeds.omegaRadiansPerSecond = MathUtil.clamp(speeds.omegaRadiansPerSecond, -slowMaxRotSpeed.get(),
                        slowMaxRotSpeed.get());
            }

            drive.runVelocity(speeds);
            Logger.recordOutput("PointControllerDist",
                    drive.getPose().getTranslation().getDistance(targetPose.getTranslation()));
        }, drive).finallyDo(() -> {
            pointControllerConverged = false;
            pointControllerRotConverged = false;
            drive.stop();
        }).withName("Point Control (Slow)");
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(
                        () -> {
                            drive.runCharacterization(0.0);
                        },
                        drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            drive.runCharacterization(voltage);
                            velocitySamples.add(drive.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        drive)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                () -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                            }
                                            double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius)
                                                    / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(Units.metersToInches(wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    public static Trajectory generateTrajectory(Drive drive, Pose2d start, Pose2d end) {
        try {
            Path path = DriveConstants.pathfinder.generatePath(start, end);
            TrajectoryConfig config = getTrajectoryConfig(drive, path);
            return path.asTrajectory(config);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to generate path: " + start + " to " + end,
                    e.getStackTrace());
            return null;
        }
    }

    private static Trajectory generateTrajectory(Drive drive, Pose2d end) {
        return generateTrajectory(drive, drive.getPose(), end);
    }

    private static TrajectoryConfig getTrajectoryConfig(Drive drive, Path path) {
        TrajectoryConfig config = new TrajectoryConfig(pathfindingMaxSpeed.get(),
                DriverStation.isAutonomous() ? pathfindingMaxAccelAuto.get() : pathfindingMaxAccel.get());

        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getVelocity(), drive.getRotation());
        Vector velocity = new Vector(chassisSpeed.vxMetersPerSecond,
                chassisSpeed.vyMetersPerSecond);
        Vertex start = path.getStart();
        Vertex nextWaypoint = path.size() > 0 ? path.get(0) : path.getTarget();
        Vector pathDir = start.createVectorTo(nextWaypoint).normalize();
        double speed = velocity.dotProduct(pathDir);

        config.setStartVelocity(speed);
        // config.setEndVelocity(Math.min(Math.abs(pointController.getXController().getP()
        // * RobotContainer.getInstance().buttonBoard.coralReefDistThreshold.get()),
        // slowMaxSpeed.get()));
        config.setEndVelocity(0);

        config.setKinematics(drive.getKinematics());
        // config.setStartVelocity(10);
        return config;
    }
}
