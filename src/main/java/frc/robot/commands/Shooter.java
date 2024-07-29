package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter;

public class Shooter extends Command{
    public final shooter m_shooter;
    public Timer timer;
    public double timeEnds;
    public double power = 1;

    // public Shooter(shooter shooterSubsystem) {
    //     m_shooter = shooterSubsystem;
    // }

    public Shooter(shooter shooterSubsystem, double power) {
        m_shooter = shooterSubsystem;
        this.power = power;
    }

    // Autonomous
    public Shooter(shooter shooterSubsystem, double power, double time) {
        m_shooter = shooterSubsystem;
        this.power = power;
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
        m_shooter.shoot(power);
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
