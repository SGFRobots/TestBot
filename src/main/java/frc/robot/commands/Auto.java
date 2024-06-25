// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.drivetrain;
import frc.robot.commands.AutoDrive;

public final class Auto extends SequentialCommandGroup{
  public drivetrain m_drive;
  
  /** Example static factory for an autonomous command. */
  public Auto(drivetrain drive) {
    addCommands(
      new AutoDrive(drive, 2, 0),
      new AutoDrive(drive, 0, 90),
      new AutoDrive(drive, 5, -35),
      new AutoDrive(drive, -1, 0)
    );
  }
}
