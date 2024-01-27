package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private boolean finished;

    public RevMotorCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }
}

    @Override
    public void initialize() {
        finished = false;

    }

    @Override
    public void execute() {
        if (NoteLocaiton()){
            m_shooterSubsystem.revShooter(ShooterConstants.motorSpeed);
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.revShooter(0);
    }
    @Ovveride
    public boolean isFinished() {

    }


