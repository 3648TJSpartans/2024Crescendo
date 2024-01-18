package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommandGroup extends SequentialCommandGroup {
    private final Supplier<Boolean> m_shootButton;
    private final ShooterSubsystem shooterSubsystem;

    public ShooterCommandGroup(ShooterSubsystem m_shooterSubsystem, Supplier<Boolean> m_shootButton) {
        shooterSubsystem = m_shooterSubsystem;
        this.m_shootButton = m_shootButton;
        // addRequirements(m_shooterSubsystem);

        addCommands(
                new RevMotorCommand(m_shooterSubsystem, m_shootButton),
                new ShootCommand(m_shooterSubsystem, m_shootButton));

    }

}
