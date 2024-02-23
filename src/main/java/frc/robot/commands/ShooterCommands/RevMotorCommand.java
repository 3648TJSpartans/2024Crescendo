package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_topSpeed;
    private final double m_bottomSpeed;

    public RevMotorCommand(ShooterSubsystem shooterSubsystem, double topSpeed, double bottomSpeed) {
        m_shooterSubsystem = shooterSubsystem;
        m_topSpeed = topSpeed;
        m_bottomSpeed = bottomSpeed;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setShooterVelocity(m_topSpeed, m_bottomSpeed);

    }

    @Override
    public void end(boolean interrupted) {

    }

}
