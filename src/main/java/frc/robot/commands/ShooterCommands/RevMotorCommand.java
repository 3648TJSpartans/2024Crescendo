package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private boolean m_note;

    public RevMotorCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_note = true;

    }

    @Override
    public void execute() {
        if (m_shooterSubsystem.NoteLocation()) {
            m_shooterSubsystem.revShooter(ShooterConstants.motorSpeed);
            m_note = false;
        }
    }

    @Override
    public boolean isFinished() {
        m_shooterSubsystem.revShooter(0);
        return m_note;
    }

}

// public void end(boolean interrupted) {
// }

// False is beam broken or object detected
// True is beam unbroken