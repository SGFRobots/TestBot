package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Speed control of robot
public class Speed extends Command {
    public final drivetrain m_drive;
    public final Slow slowMode;
    public final Fast fastMode;

    public Speed(drivetrain drive) {
        // create instances of both classes/modes
        m_drive = drive;
        slowMode = new Slow();
        fastMode = new Fast();
    }

    // Slow Mode
    public class Slow extends Command {
        public Slow() {}

        @Override
        // Start slow mode
        public void initialize() {
            m_drive.slow = true;
        }
    
        @Override
        public void execute() {}
        
        @Override
        // End slow mode
        public void end(boolean interrupted) {
            m_drive.slow = false;
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }
    }

    // Fast Mode
    public class Fast extends Command {
        public Fast() {}

        @Override
        // Start fast mode
        public void initialize() {
            m_drive.fast = true;
        }
    
        @Override
        public void execute() {}
        
        // Ends fast mode
        @Override
        public void end(boolean interrupted) {
            m_drive.fast = false;
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }
    }
}
