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

    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.revShooter(ShooterConstants.motorSpeed);
        m_shooterSubsystem.moveShooterIntake(ShooterConstants.beltMotorSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.revShooter(0);
        m_shooterSubsystem.moveShooterIntake(0);
    }
}
