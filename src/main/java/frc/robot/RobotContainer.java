// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Limelight imports

// Power Distribution Board imports
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.commands.DrivewithJoysticks;
import frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.XboxController;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final drivetrain m_drive = new drivetrain();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final XboxController m_controller = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_drive.setDefaultCommand(new DrivewithJoysticks(m_drive));
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // TODO understand how to do this
  }


  public void getAutonomousCommand() {
    // An example command will be run in autonomous
  }
}
