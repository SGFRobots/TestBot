// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private RobotContainer m_robotContainer;
  public static String GameStage = "";
  
  private final Timer m_timer = new Timer();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    GameStage = "init";
    SmartDashboard.putString("GameStage: ", GameStage);
    Mechanism2d mech = new Mechanism2d(3, 3);
    SmartDashboard.putData("Mech2d", mech);
    m_robotContainer = new RobotContainer();

    // Calculate trajectory from one point to another, given start, middle, end points and speed
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    GameStage = "disabled";
  }
  
  @Override
  public void disabledPeriodic() {}
  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    GameStage = "auto";
    SmartDashboard.putString("GameStage: ", GameStage);
    m_timer.restart();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // double elapsed = m_timer.get();
    // System.out.println(elapsed);
  }
  
  @Override
  public void teleopInit() {
    GameStage = "teleop";
    SmartDashboard.putString("GameStage: ", GameStage);
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // RobotContainer.m_driveCommand.execute();
    SmartDashboard.putBoolean("Slow Mode", RobotContainer.m_drive.slow);
    SmartDashboard.putBoolean("Fast Mode", RobotContainer.m_drive.fast);
    SmartDashboard.putBoolean("Moving", !RobotContainer.m_controller2.getTriangleButton());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    GameStage = "simulation";
    SmartDashboard.putString("GameStage: ", GameStage);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    RobotContainer.m_drive.simulationPeriodic();
  }
}
