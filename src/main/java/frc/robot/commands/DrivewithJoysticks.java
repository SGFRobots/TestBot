package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
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
        // m_drive.arcadeDrive();
        double xSpeed = -drivetrain.driveFilter.calculate(RobotContainer.m_controller.getLeftY()) * 3.0;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        double rot = -drivetrain.turnFilter.calculate(RobotContainer.m_controller.getRightX()) * Math.PI;
        m_drive.drive(xSpeed, rot);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
