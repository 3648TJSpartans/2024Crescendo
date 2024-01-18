package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommandGroup extends SequentialCommandGroup {
    private final Supplier<Boolean> m_ShootButton;
    private final ShooterSubsystem ShooterSubsystem;

    public ShooterCommandGroup(ShooterSubsystem m_ShooterSubsystem, Supplier<Boolean> m_ShootButton) {
        ShooterSubsystem = m_ShooterSubsystem;
        this.m_ShootButton = m_ShootButton;
        // addRequirements(m_ShooterSubsystem);

        addCommands(
                new RevMotorCommand(m_ShooterSubsystem, m_ShootButton),
                new ShootCommand(m_ShooterSubsystem, m_ShootButton));

    }

}
