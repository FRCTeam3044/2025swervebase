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

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceUtil;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
        private final Drive drive;
        private final Vision vision;
        private SwerveDriveSimulation driveSimulation = null;

        // Controller
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final CommandXboxController operatorController = new CommandXboxController(1);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        public final StateMachine stateMachine;

        public static Field2d fieldSim = new Field2d();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                drive = new Drive(
                                                new GyroIONavX(),
                                                new ModuleIOSpark(0),
                                                new ModuleIOSpark(1),
                                                new ModuleIOSpark(2),
                                                new ModuleIOSpark(3));

                                this.vision = new Vision(drive);
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
                                                                driveSimulation::getSimulatedDriveTrainPose));
                                vision.setPoseSupplier(driveSimulation::getSimulatedDriveTrainPose);
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
                                break;
                }

                AllianceUtil.setRobot(drive::getPose);

                stateMachine = new StateMachine(driverController, operatorController, drive);

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

                // Configure the button bindings
                configureButtonBindings();
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

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return DriveCommands.goToPointJoystickRot(drive, new Pose2d(10, 7, new Rotation2d()), () -> 0.0);
                // return autoChooser.get();
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
                                "FieldSimulation/Notes",
                                SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
        }

        public void displaySimFieldToSmartDashboard() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                fieldSim.setRobotPose(driveSimulation.getSimulatedDriveTrainPose());
                fieldSim.getObject("notes")
                                .setPoses(SimulatedArena.getInstance().getGamePiecesByType("Note").stream().map((p) -> {
                                        return new Pose2d(p.getTranslation().getX(), p.getTranslation().getY(),
                                                        new Rotation2d());
                                }).toArray(Pose2d[]::new));
                SmartDashboard.putData("Field", fieldSim);
        }
}
