// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.constants.DriveTrainConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO or NEO 550), and
 * analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware configurations
 * (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward motion on the drive motor
 * will propel the robot forward) and copy the reported values from the absolute encoders using AdvantageScope. These
 * values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSpark implements ModuleIO {
    private final SparkMax driveSparkMax;
    private final SparkMax turnSparkMax;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;
    private final Queue<Double> drivePositionInput;
    private final Queue<Double> steerRelativeEncoderPositionUngeared;

    private final boolean isTurnMotorInverted = true;

    public ModuleIOSpark(int index) {
        switch (index) {
            case 0 -> {
                driveSparkMax = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
                turnSparkMax = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
            }
            case 1 -> {
                driveSparkMax = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);
                turnSparkMax = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);
            }
            case 2 -> {
                driveSparkMax = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);
                turnSparkMax = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);
            }
            case 3 -> {
                driveSparkMax = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
                turnSparkMax = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
            }
            default -> throw new RuntimeException("Invalid module index");
        }

        drivingEncoder = driveSparkMax.getEncoder();
        turningEncoder = turnSparkMax.getAbsoluteEncoder();

        drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        driveConfig.smartCurrentLimit(40).voltageCompensation(12.0).idleMode(SparkBaseConfig.IdleMode.kBrake);
        driveConfig.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);
        driveConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY));

        turnConfig
                .inverted(isTurnMotorInverted)
                .smartCurrentLimit(30)
                .voltageCompensation(12.0)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        turnConfig.encoder.quadratureMeasurementPeriod(10).quadratureAverageDepth(2);
        driveConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY));

        driveSparkMax.configure(
                driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        turnSparkMax.configure(
                turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        this.drivePositionInput = OdometryThread.registerInput(driveEncoder::getPosition);
        this.steerRelativeEncoderPositionUngeared = OdometryThread.registerInput(steerRelativeEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions = driveEncoder.getPosition() / DRIVE_GEAR_RATIO;
        final double RPTO_REVOLUTIONS_PER_SECOND = 1.0 / 60.0;
        inputs.driveWheelFinalVelocityRevolutionsPerSec =
                driveEncoder.getVelocity() / DRIVE_GEAR_RATIO * RPTO_REVOLUTIONS_PER_SECOND;

        inputs.driveMotorAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveMotorCurrentAmps = driveSparkMax.getOutputCurrent();

        inputs.steerFacing = Rotation2d.fromRotations(steerRelativeEncoder.getPosition() / STEER_GEAR_RATIO)
                .minus(steerRelativePositionEncoderOffset);
        inputs.steerVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(steerRelativeEncoder.getVelocity() / STEER_GEAR_RATIO);

        inputs.steerMotorAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.steerMotorCurrentAmps = turnSparkMax.getOutputCurrent();

        inputs.odometryDriveWheelRevolutions = drivePositionInput.stream()
                .mapToDouble(value -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
                .toArray();
        drivePositionInput.clear();
        inputs.odometrySteerPositions = steerRelativeEncoderPositionUngeared.stream()
                .map(value ->
                        Rotation2d.fromRotations(value / STEER_GEAR_RATIO).minus(steerRelativePositionEncoderOffset))
                .toArray(Rotation2d[]::new);
        steerRelativeEncoderPositionUngeared.clear();
    }

    private Rotation2d steerRelativePositionEncoderOffset = new Rotation2d();

    @Override
    public void calibrate() {
        final Rotation2d steerActualFacing = Rotation2d.fromDegrees(turnSparkMax.get)
                .minus(absoluteEncoderOffset);
        final Rotation2d relativeEncoderReportedFacing =
                Rotation2d.fromRotations(steerRelativeEncoder.getPosition() / STEER_GEAR_RATIO);
        /* reported - offset = actual, so offset = reported - actual */
        steerRelativePositionEncoderOffset = relativeEncoderReportedFacing.minus(steerActualFacing);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkMax.set(volts / RobotController.getBatteryVoltage());
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
        turnSparkMax.set(powerPercent);
    }
}
