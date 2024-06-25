package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain;

public class AutoDrive extends Command {
    drivetrain m_drive;
    double x;
    double y;
    double degrees;
    double drive = 0.5;
    double turn = 0.3;
    double distance;
    double rotation;

    public AutoDrive(drivetrain drive, double distance, double rotation) {
        m_drive = drive;
        this.distance = distance;
        this.rotation = rotation;
        if(distance < 0) {
            this.distance *= -1;
            this.drive *= -1;
        }
        if(rotation < 0) {
            this.rotation *= -1;
            this.turn *= -1;
        }
        
        //negative is right and backwards
    }

    @Override
    public void initialize() {
        x = m_drive.getPose().getX();
        y = m_drive.getPose().getY();
        degrees = m_drive.getPose().getRotation().getDegrees();
    }

    @Override
    public void execute() {
        m_drive.autoDrive(drive, turn);
    }

    @Override 
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(m_drive.getPose().getRotation().getDegrees() - degrees) >= rotation) {
            turn = 0;
        }
        if(Math.sqrt(Math.pow(m_drive.getPose().getX() - x,2) + Math.pow(m_drive.getPose().getY() - y,2)) >= distance) {
            drive = 0;
        }
        return ((drive == 0) && (turn == 0));
    }
}
