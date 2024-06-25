// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

// Subsystems and commands
import frc.robot.commands.DrivewithJoysticks;
import frc.robot.subsystems.drivetrain;
import frc.robot.commands.Speed;
import frc.robot.commands.Auto;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static drivetrain m_drive;
  public static DrivewithJoysticks m_driveCommand;
  public static Speed speedControl;


  // Instances of controllers
  public static final XboxController m_controller = new XboxController(0);
  public static final PS4Controller m_controller2 = new PS4Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems and commands
    m_drive = new drivetrain();
    m_drive.setDefaultCommand(new DrivewithJoysticks(m_drive));
    speedControl = new Speed(m_drive);

    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    // SlowMode
    new JoystickButton(m_controller2, PS4Controller.Button.kL1.value).whileTrue(speedControl.slowMode);
    new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value).whileTrue(speedControl.slowMode);
    
    // FastMode
    new JoystickButton(m_controller2, PS4Controller.Button.kR1.value).whileTrue(speedControl.fastMode);
    new JoystickButton(m_controller, XboxController.Button.kRightBumper.value).whileTrue(speedControl.fastMode);
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Auto(m_drive);
  }
}
