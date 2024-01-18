package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final Supplier<Boolean> m_shootButton;
    private final ShooterSubsystem shooterSubsystem;

    public RevMotorCommand(ShooterSubsystem m_shooterSubsystem, Supplier<Boolean> m_shootButton) {
        shooterSubsystem = m_shooterSubsystem;
        this.m_shootButton = m_shootButton;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_shootButton.get()) {
            shooterSubsystem.revShooter();
        }

    }

}
