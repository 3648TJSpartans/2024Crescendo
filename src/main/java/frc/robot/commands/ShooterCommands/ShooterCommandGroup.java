package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommandGroup extends SequentialCommandGroup {
    private final ShooterSubsystem m_shooterSubsystem;

    public ShooterCommandGroup(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;

        addCommands(
                new RevMotorCommand(m_shooterSubsystem),
                new WaitCommand(ShooterConstants.idleTime),
                new ShootCommand(m_shooterSubsystem));
    }
}
