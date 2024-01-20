package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;

    public RevMotorCommand(ShooterSubsystem m_shooterSubsystem) {
        this.m_shooterSubsystem = m_shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        {
            m_shooterSubsystem.revShooter(ShooterConstants.beltMotorSpeed);

        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.revShooter(0);
    }

}
