package frc.robot.commands.ShooterCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommandGroup extends SequentialCommandGroup {
    private final ShooterSubsystem m_shooterSubsystem;

    public ShooterCommandGroup(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;

        addCommands(
                new RevMotorCommand(m_shooterSubsystem).withTimeout(.5),
                new WaitCommand(ShooterConstants.revIdleTime),
                new ShootCommand(m_shooterSubsystem).withTimeout(2),
                new WaitCommand(ShooterConstants.shootTime));
    }

}
