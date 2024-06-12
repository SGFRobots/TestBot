package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain;

public class DrivewithJoysticks extends Command {
    drivetrain m_drive;
    
    public DrivewithJoysticks(drivetrain drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_drive.arcadeDrive();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
