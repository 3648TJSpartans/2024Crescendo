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

    public ShootCommand(ShooterSubsystem m_shooterSubsystem) {
        this.m_shooterSubsystem = m_shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.moveShooterIntake(ShooterConstants.motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.moveShooterIntake(0);
    }
}
