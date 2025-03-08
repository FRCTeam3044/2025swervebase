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

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.subsystems.LEDs.LEDsIO;
import frc.robot.subsystems.LEDs.LEDsIORio;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.endEffector.EndEffectorIOSpark;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderIO;
import frc.robot.subsystems.shoulder.ShoulderIOSim;
import frc.robot.subsystems.shoulder.ShoulderIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceUtil;
import frc.robot.util.bboard.BBoardIOReal;
import frc.robot.util.bboard.BBoardIOSim;
import frc.robot.util.bboard.ButtonBoard;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        public final Drive drive;
        public final ButtonBoard buttonBoard;
        private final Vision vision;
        private final Elevator elevator;
        public final Shoulder shoulder;
        private final EndEffector endEffector;
        private final LEDs LEDs;
        private final Climber climber;
        private SwerveDriveSimulation driveSimulation = null;

        // Controller
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final CommandXboxController operatorController = new CommandXboxController(1);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        public final StateMachine stateMachine;

        public static Field2d fieldSim = new Field2d();

        private final Mechanism2d mech;
        private final MechanismRoot2d root;
        private final MechanismLigament2d elevatorSim;
        private final MechanismLigament2d shoulderSim;

        private static RobotContainer instance;

        public static RobotContainer getInstance() {
                return instance;
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                instance = this;
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                drive = new Drive(
                                                new GyroIONavX(),
                                                new ModuleIOSpark(0),
                                                new ModuleIOSpark(1),
                                                new ModuleIOSpark(2),
                                                new ModuleIOSpark(3));
                                this.vision = new Vision(drive,
                                                new VisionIOPhotonVision(VisionConstants.camera0Name,
                                                                VisionConstants.robotToCamera0),
                                                new VisionIOPhotonVision(VisionConstants.camera1Name,
                                                                VisionConstants.robotToCamera1),
                                                new VisionIOPhotonVision(VisionConstants.camera2Name,
                                                                VisionConstants.robotToCamera2));
                                shoulder = new Shoulder(new ShoulderIOSpark());
                                elevator = new Elevator(new ElevatorIOSpark(), shoulder::inDangerZone);
                                shoulder.setElevatorNotAtTargetSupplier(elevator::notAtTarget);
                                endEffector = new EndEffector(new EndEffectorIOSpark());
                                buttonBoard = new ButtonBoard(new BBoardIOReal());
                                LEDs = new LEDs(new LEDsIORio());
                                climber = new Climber(new ClimberIOSpark());
                                // vision = new Vision(drive, new VisionIO() {
                                // }, new VisionIO() {
                                // });
                                // shoulder = new Shoulder(new ShoulderIO() {
                                // });
                                // elevator = new Elevator(new ElevatorIO() {
                                // }, shoulder::inDangerZone);
                                // endEffector = new EndEffector(new EndEffectorIO() {
                                // });
                                // buttonBoard = new ButtonBoard(new BBoardIOReal());
                                // LEDs = new LEDs(new LEDsIO() {
                                // });
                                // climber = new Climber(new ClimberIO() {

                                // });
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                this.driveSimulation = new SwerveDriveSimulation(
                                                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                                // add the simulated drivetrain to the simulation field
                                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(
                                                new GyroIOSim(driveSimulation.getGyroSimulation()),
                                                new ModuleIOSim(driveSimulation.getModules()[0]),
                                                new ModuleIOSim(driveSimulation.getModules()[1]),
                                                new ModuleIOSim(driveSimulation.getModules()[2]),
                                                new ModuleIOSim(driveSimulation.getModules()[3]));
                                vision = new Vision(
                                                drive,
                                                new VisionIOPhotonVisionSim(
                                                                camera0Name, robotToCamera0,
                                                                driveSimulation::getSimulatedDriveTrainPose),
                                                new VisionIOPhotonVisionSim(
                                                                camera1Name, robotToCamera1,
                                                                driveSimulation::getSimulatedDriveTrainPose),
                                                new VisionIOPhotonVisionSim(
                                                                camera2Name, robotToCamera2,
                                                                driveSimulation::getSimulatedDriveTrainPose));
                                vision.setPoseSupplier(driveSimulation::getSimulatedDriveTrainPose);
                                shoulder = new Shoulder(new ShoulderIOSim());
                                elevator = new Elevator(new ElevatorIOSim(), shoulder::inDangerZone);
                                shoulder.setElevatorNotAtTargetSupplier(elevator::notAtTarget);
                                endEffector = new EndEffector(new EndEffectorIOSim(driverController.getHID()));
                                buttonBoard = new ButtonBoard(
                                                Constants.simButtonBoard ? new BBoardIOSim() : new BBoardIOReal());
                                LEDs = new LEDs(new LEDsIORio());
                                climber = new Climber(new ClimberIO() {

                                });
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                vision = new Vision(drive, new VisionIO() {
                                }, new VisionIO() {
                                });
                                shoulder = new Shoulder(new ShoulderIO() {
                                });
                                elevator = new Elevator(new ElevatorIO() {
                                }, shoulder::inDangerZone);
                                shoulder.setElevatorNotAtTargetSupplier(elevator::notAtTarget);
                                endEffector = new EndEffector(new EndEffectorIO() {
                                });
                                buttonBoard = new ButtonBoard(new BBoardIOReal());
                                LEDs = new LEDs(new LEDsIORio());
                                climber = new Climber(new ClimberIO() {

                                });
                                break;
                }

                AllianceUtil.setRobot(drive::getPose);

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices");

                // Set up SysId routines
                autoChooser.addOption(
                                "Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                autoChooser.addOption(
                                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                autoChooser.addOption("Elevator SysID (Quasistatic Forward)",
                                elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Elevator SysId (Quasistatic Reverse)",
                                elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption("Elevator SysId (Dynamic Forward)",
                                elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Elevator SysId (Dynamic Reverse)",
                                elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                autoChooser.addOption("Shoulder SysID (Quasistatic Forward)",
                                shoulder.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Shoulder SysId (Quasistatic Reverse)",
                                shoulder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption("Shoulder SysId (Dynamic Forward)",
                                shoulder.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Shoulder SysId (Dynamic Reverse)",
                                shoulder.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                stateMachine = new StateMachine(driverController, operatorController, buttonBoard, autoChooser, drive,
                                elevator, shoulder, endEffector, LEDs, climber);

                // Configure the button bindings
                configureButtonBindings();

                mech = new Mechanism2d(3, 3);
                root = mech.getRoot("elevator", 1.5, 0);
                elevatorSim = root
                                .append(new MechanismLigament2d("elevator", elevator.getElevatorHeight(), 90));
                shoulderSim = elevatorSim
                                .append(new MechanismLigament2d("shoulder", Inches.of(15.75).in(Meters), -90, 6.0,
                                                new Color8Bit(Color.kPurple)));
                shoulderSim.append(
                                new MechanismLigament2d("endEffector", Inches.of(13).in(Meters), 90.0, 6.0,
                                                new Color8Bit(Color.kPurple)));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Default command, normal field-relative drive

        }

        public void resetSimulationField() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;
                driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().resetFieldForAuto();
        }

        public void displaySimFieldToAdvantageScope() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;
                Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Coral",
                                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
                Logger.recordOutput(
                                "FieldSimulation/Algae",
                                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        }

        public void displaySimFieldToSmartDashboard() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                fieldSim.setRobotPose(driveSimulation.getSimulatedDriveTrainPose());
                fieldSim.getObject("algae")
                                .setPoses(SimulatedArena.getInstance().getGamePiecesByType("Algae").stream()
                                                .map((p) -> {
                                                        return new Pose2d(p.getTranslation().getX(),
                                                                        p.getTranslation().getY(),
                                                                        new Rotation2d());
                                                }).toArray(Pose2d[]::new));
                fieldSim.getObject("coral")
                                .setPoses(SimulatedArena.getInstance().getGamePiecesByType("Coral").stream()
                                                .map((p) -> {
                                                        return new Pose2d(p.getTranslation().getX(),
                                                                        p.getTranslation().getY(),
                                                                        new Rotation2d(p.getRotation().getZ()));
                                                }).toArray(Pose2d[]::new));

                SmartDashboard.putData("Field", fieldSim);
        }

        public void updateMechanism() {
                elevatorSim.setLength(elevator.getElevatorHeight());
                shoulderSim.setAngle(Math.toDegrees(shoulder.getShoulderAngle()) - 90);
                SmartDashboard.putData("Mech2d", mech);
        }

        public void shootCoral() {
                SimulatedArena.getInstance()
                                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                                // Obtain robot position from drive simulation
                                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                                                new Translation2d(-0.35, 0),
                                                // Obtain robot speed from drive simulation
                                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                                // Obtain robot facing from drive simulation
                                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                                // The height at which the coral is ejected
                                                Meters.of(elevator.getElevatorHeight() + 0.45),
                                                // The initial speed of the coral
                                                MetersPerSecond.of(2),
                                                // The coral is ejected at a 35-degree slope
                                                Radians.of(shoulder.getShoulderAngle()).plus(Degree.of(90))));
        }
}
