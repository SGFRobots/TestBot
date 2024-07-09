package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter;

public class Shooter extends Command{
    public final shooter m_shooter;

    public Shooter(shooter shooterSubsystem) {
        m_shooter = shooterSubsystem;
    }

    @Override
    public void initialize() {}

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
        return false;
    }
}
