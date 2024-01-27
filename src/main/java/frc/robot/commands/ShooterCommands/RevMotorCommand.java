package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private boolean Finished;

    public RevMotorCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        Finished = true;

    }

    @Override
    public void execute() {
        if (m_shooterSubsystem.NoteLocation()) {
            m_shooterSubsystem.revShooter(ShooterConstants.motorSpeed);
            Finished = false;
        }
    }

    @Override
    public void end(boolean isFinished) {
        m_shooterSubsystem.revShooter(0);
    }

}

// False is beam broken or object fully detected
// True is beam unbroken