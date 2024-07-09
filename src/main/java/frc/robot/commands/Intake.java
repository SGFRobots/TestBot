package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class Intake extends Command {
    public intake m_intake;

    public Intake(intake intakeSubsystem) {
        m_intake = intakeSubsystem;
    }

    @Override
    public void initialize() {}

    @Override
    // Turn intake on
    public void execute() {
        m_intake.scoop();
    }

    @Override
    // Stop intake
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
