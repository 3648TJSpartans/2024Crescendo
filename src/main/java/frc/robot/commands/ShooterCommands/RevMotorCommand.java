package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_shootSpeed;

    public RevMotorCommand(ShooterSubsystem shooterSubsystem, double shootSpeed) {
        m_shooterSubsystem = shooterSubsystem;
        m_shootSpeed = shootSpeed;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.revShooter(m_shootSpeed);

    }

    @Override
    public void end(boolean interrupted) {

    }

}
