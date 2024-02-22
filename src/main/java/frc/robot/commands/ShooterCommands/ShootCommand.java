package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_shooterSpeed1;
    private final double m_shooterSpeed2;
    private final double m_beltSpeed;

    public ShootCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed1, double shooterSpeed2,
            double beltSpeed) {
        m_beltSpeed = beltSpeed;
        m_shooterSpeed1 = shooterSpeed1;
        m_shooterSpeed2 = shooterSpeed2;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setShooterVelocity(m_shooterSpeed1, m_shooterSpeed2);
        m_shooterSubsystem.setBeltSpeed(m_beltSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setShooterVelocity(0, 0);
        m_shooterSubsystem.setBeltSpeed(0);
    }
}
