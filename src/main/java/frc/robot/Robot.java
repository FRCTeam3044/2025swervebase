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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autos.AutoStep;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoTargetUtils.Reef;
import frc.robot.util.AutoTargetUtils.Reef.CoralLevel;
import frc.robot.util.AutoTargetUtils.Reef.CoralReefLocation;
import frc.robot.util.bboard.BBoardIOAuto;
import me.nabdev.oxconfig.OxConfig;

import java.util.ArrayList;
import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public List<AutoStep> currentAuto;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer.
    robotContainer = new RobotContainer();

    OxConfig.initialize();
    robotContainer.stateMachine.onStartup();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    robotContainer.buttonBoard.periodic(robotContainer.drive);
    robotContainer.stateMachine.periodic();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    Logger.recordOutput("Alliance", AllianceUtil.getAlliance().toString());
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    // PathfindingDebugUtils.drawLines("Field Map",
    // DriveConstants.pathfinder.visualizeEdges(),
    // DriveConstants.pathfinder.visualizeVertices());

    // PathfindingDebugUtils.drawLines("Field Map Inflated",
    // DriveConstants.pathfinder.visualizeEdges(),
    // DriveConstants.pathfinder.visualizeInflatedVertices());
    ArrayList<Double> distances = new ArrayList<>();
    for (CoralReefLocation location : Reef.CoralReefLocation.values()) {
      Pose2d pose = Reef.coral(location, CoralLevel.L1);
      distances.add(pose.getX());
      distances.add(pose.getY());
      distances.add(pose.getRotation().getDegrees());
    }
    double[] dist = distances.stream().mapToDouble(Double::doubleValue).toArray();
    SmartDashboard.putNumberArray("Coral Reef Locations", dist);

    robotContainer.updateMechanism();
    robotContainer.displayRealFieldToSmartDashboard();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.resetSimulationField();
    AllianceUtil.setAlliance();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    AllianceUtil.setAlliance();
  }

  private List<AutoStep> auto;
  private int currentAutoStep = 0;
  private BBoardIOAuto boardIOAuto = new BBoardIOAuto();

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    auto = robotContainer.autos.autoChooser.get();
    robotContainer.buttonBoard.setAutoIO(boardIOAuto);
    currentAutoStep = 0;
    AllianceUtil.setAlliance();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    AllianceUtil.setIfUnknown();
    if (currentAutoStep >= auto.size()) {
      return;
    }
    AutoStep current = auto.get(currentAutoStep);
    if (current.exit().getAsBoolean()) {
      currentAutoStep++;
    }
    boardIOAuto.setSelectPressed(current.buttons());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    robotContainer.buttonBoard.normalIO();
    AllianceUtil.setAlliance();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    AllianceUtil.setIfUnknown();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    robotContainer.buttonBoard.normalIO();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    robotContainer.displaySimFieldToAdvantageScope();
    robotContainer.displaySimFieldToSmartDashboard();
  }
}
