package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter;

public class Shooter extends Command{
    public final shooter m_shooter;
    public Timer timer;
    public double timeEnds;

    public Shooter(shooter shooterSubsystem) {
        m_shooter = shooterSubsystem;
    }

    // Autonomous
    public Shooter(shooter shooterSubsystem, double time) {
        m_shooter = shooterSubsystem;
        // Set timer
        timeEnds = time;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        // Start timer - Autonomous
        if(timer != null) {
            timer.restart();
        }
    }

    @Override
    // Shoot
    public void execute() {
        m_shooter.shoot();
    }

    @Override
    // Stop
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        // Ends when time runs out - Autonomous
        if(timer != null) {
            if(timer.get() >= timeEnds) {
                return true;
            }
        }
        return false;
    }

}
