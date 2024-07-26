// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

// Subsystems and commands
import frc.robot.commands.DrivewithJoysticks;
import frc.robot.subsystems.drivetrain;
import frc.robot.commands.Speed;
import frc.robot.commands.Auto;
import frc.robot.subsystems.intake;
import frc.robot.commands.Intake;
import frc.robot.subsystems.shooter;
import frc.robot.commands.Shooter;
// import frc.robot.subsystems.Swerve.SwerveSubsystem;
// import frc.robot.commands.SwerveJoystick;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static drivetrain m_drive;
  public static DrivewithJoysticks m_driveCommand;
  public static Speed speedControl;
  public static intake m_intake;
  public static shooter m_shooter;

  // Instances of controllers
  public static final XboxController m_controller = new XboxController(0);

  // Swerve subsystem
  // private final SwerveSubsystem swerveSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems and commands
    m_drive = new drivetrain();
    // swerveSubsystem = new SwerveSubsystem();
    // swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem, m_controller));
    m_drive.setDefaultCommand(new DrivewithJoysticks(m_drive));
    speedControl = new Speed(m_drive);
    m_intake = new intake();
    m_shooter = new shooter();


    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    // SlowMode
    new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value).whileTrue(speedControl.slowMode);
    
    // FastMode
    new JoystickButton(m_controller, XboxController.Button.kRightBumper.value).whileTrue(speedControl.fastMode);

    // Intake
    new JoystickButton(m_controller, XboxController.Button.kX.value).toggleOnTrue(new Intake(m_intake));

    // Shooter
    new JoystickButton(m_controller, XboxController.Button.kB.value).toggleOnTrue(new Shooter(m_shooter));
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Auto(m_drive);
  }
}
