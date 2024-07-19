// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain;

public final class Auto extends ParallelCommandGroup {
  public drivetrain m_drive;
  
  // Call commands during autonomous
  public Auto(drivetrain drive) {

    // Run commands while Intake is always on
    addCommands(
      new Intake(RobotContainer.m_intake),
      new AutoCommands(drive)
    );

    }
    
    // Run commands in order
    public class AutoCommands extends SequentialCommandGroup {
      public AutoCommands(drivetrain drive) {
        addCommands(
          new AutoDrive(drive, 2, 0),
          new Shooter(RobotContainer.m_shooter, 1.0),
          new AutoDrive(drive, 0, 90),
          new AutoDrive(drive, 5, -35),
          new AutoDrive(drive, -1, 0),
          new AutoDrive(drive, 2, 35)
      );
    }
  }

}
