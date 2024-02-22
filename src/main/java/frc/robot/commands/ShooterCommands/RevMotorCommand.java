package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_shootSpeed1;
    private final double m_shootSpeed2;

    public RevMotorCommand(ShooterSubsystem shooterSubsystem, double shootSpeed1, double shootSpeed2) {
        m_shooterSubsystem = shooterSubsystem;
        m_shootSpeed1 = shootSpeed1;
        m_shootSpeed2 = shootSpeed2;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setShooterVelocity(m_shootSpeed1, m_shootSpeed2);

    }

    @Override
    public void end(boolean interrupted) {

    }

}
