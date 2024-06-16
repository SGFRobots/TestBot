package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain;


public class DrivewithJoysticks extends Command {
    drivetrain m_drive;
    
    public DrivewithJoysticks(drivetrain drive) {
        // create the drive train system. didn't we already do this?
        m_drive = drive;
        addRequirements(m_drive); // what??
    }

    @Override
    public void initialize() {}

    @Override
    // runs every time the thingy passes (milisecond or something like that)
    public void execute() {
        if(Robot.GameStage == "teleop") {
            m_drive.drive();
            
        }
    }

    public void auto(double elapsed) {
        if(elapsed < 5) {
            // m_drive.drive(2, 0);
        } 
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
