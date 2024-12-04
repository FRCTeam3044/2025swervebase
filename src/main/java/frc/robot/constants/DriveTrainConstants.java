package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * stores the constants and PID configs for chassis because we want an all-real simulation for the chassis, the numbers
 * are required to be considerably precise
 */
public class DriveTrainConstants {
    /** numbers that needs to be changed to fit each robot TODO: change these numbers to match your robot */
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.43;

    public static final Mass ROBOT_MASS = Kilograms.of(45); // robot weight with bumpers

    /** TODO: change motor type to match your robot */
    public static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1);

    public static final DCMotor STEER_MOTOR = DCMotor.getNeo550(1);

    /** numbers imported from {@link TunerConstants} TODO: for REV chassis, replace them with actual numbers */
    public static final Distance WHEEL_RADIUS = Inches.of(1.5);

    public static final double DRIVE_GEAR_RATIO = 4.71;
    public static final double STEER_GEAR_RATIO = 46.42;

    public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.25);
    public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.25);
    public static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.025);

    /* adjust current limit */
    public static final Current DRIVE_CURRENT_LIMIT = Amps.of(40);
    public static final Current STEER_CURRENT_LIMIT = Amps.of(20);
    

    public static final Distance TRACK_WIDTH = Inches.of(26);
    public static final Distance TRACK_LENGTH = Inches.of(26);
    public static final double ROBOT_SIZE = Units.inchesToMeters(37);


    /** translations of the modules to the robot center, in FL, FR, BL, BR */
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(-TRACK_WIDTH.in(Meters) / 2, TRACK_LENGTH.in(Meters) / 2),
        new Translation2d(TRACK_WIDTH.in(Meters) / 2, TRACK_LENGTH.in(Meters) / 2),
        new Translation2d(-TRACK_WIDTH.in(Meters) / 2, -TRACK_LENGTH.in(Meters) / 2),
        new Translation2d(TRACK_WIDTH.in(Meters) / 2, -TRACK_LENGTH.in(Meters) / 2)
    };

    public static final double[] CHASSIS_ANGULAR_OFFSETS = {-Math.PI / 2, 0, Math.PI, Math.PI / 2};

    /* equations that calculates some constants for the simulator (don't modify) */
    private static final double GRAVITY_CONSTANT = 9.8;

    public static final Distance DRIVE_BASE_RADIUS = Meters.of(MODULE_TRANSLATIONS[0].getNorm());

    /* friction_force = normal_force * coefficient_of_friction */
    public static final LinearAcceleration MAX_FRICTION_ACCELERATION =
            MetersPerSecondPerSecond.of(GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION);

    /* force = torque / distance */
    public static final Force MAX_PROPELLING_FORCE = NewtonMeters.of(
                    DRIVE_MOTOR.getTorque(DRIVE_CURRENT_LIMIT.in(Amps)) * DRIVE_GEAR_RATIO)
            .divide(WHEEL_RADIUS);

    /* floor_speed = wheel_angular_velocity * wheel_radius */
    public static final LinearVelocity CHASSIS_MAX_VELOCITY =
            MetersPerSecond.of(DRIVE_MOTOR.freeSpeedRadPerSec / DRIVE_GEAR_RATIO * WHEEL_RADIUS.in(Meters));
    public static final LinearAcceleration CHASSIS_MAX_ACCELERATION =
            (LinearAcceleration) Measure.min(MAX_FRICTION_ACCELERATION, MAX_PROPELLING_FORCE.divide(ROBOT_MASS));
    public static final AngularVelocity CHASSIS_MAX_ANGULAR_VELOCITY =
            RadiansPerSecond.of(CHASSIS_MAX_VELOCITY.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));
    public static final AngularAcceleration CHASSIS_MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(
            CHASSIS_MAX_ACCELERATION.in(MetersPerSecondPerSecond) / DRIVE_BASE_RADIUS.in(Meters) * 2);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    /* for collision detection in simulation */
    public static final Distance BUMPER_WIDTH = Inches.of(30), BUMPER_LENGTH = Inches.of(30);

    // https://unacademy.com/content/upsc/study-material/physics/moment-of-inertia-of-rectangle-section/
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(ROBOT_MASS.in(Kilograms)
            * (BUMPER_WIDTH.in(Meters) * BUMPER_WIDTH.in(Meters) + BUMPER_LENGTH.in(Meters) * BUMPER_LENGTH.in(Meters))
            / 12.0);

    public static final Supplier<GyroSimulation> gyroSimulationFactory = GyroSimulation.getGeneric();

    /* dead configs, don't change them */
    public static final int ODOMETRY_CACHE_CAPACITY = 10;
    public static final double ODOMETRY_FREQUENCY = 250;
    public static final double ODOMETRY_WAIT_TIMEOUT_SECONDS = 0.02;
    public static final int SIMULATION_TICKS_IN_1_PERIOD = 5;

public static final class ModuleConstants {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth
                // will result in a
                // robot that drives faster).
                public static final int kDrivingMotorPinionTeeth = 14;

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = 0.0762;
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the bevel pinion
                public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters)
                                / kDrivingMotorReduction;

                public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction; // meters
                public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction) / 60.0; // meters per second

                public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
                public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

                public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
                public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

                public static final double kDrivingP = 0.04;
                public static final double kDrivingI = 0;
                public static final double kDrivingD = 0;
                public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
                public static final double kDrivingMinOutput = -1;
                public static final double kDrivingMaxOutput = 1;

                public static final double kTurningP = 1;
                public static final double kTurningI = 0;
                public static final double kTurningD = 0;
                public static final double kTurningFF = 0;
                public static final double kTurningMinOutput = -1;
                public static final double kTurningMaxOutput = 1;

                public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
                public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

                public static final int kDrivingMotorCurrentLimit = 40; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps
        }

}
