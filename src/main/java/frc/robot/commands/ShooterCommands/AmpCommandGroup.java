package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpCommandGroup extends SequentialCommandGroup {
    private final ShooterSubsystem m_shooterSubsystem;

    public AmpCommandGroup(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addCommands(new RevMotorCommand(m_shooterSubsystem, ShooterConstants.shooterAmpSpeed)
                .withTimeout(ShooterConstants.revAmpTime),
                new ShootCommand(m_shooterSubsystem, ShooterConstants.shooterAmpSpeed, ShooterConstants.beltAmpSpeed)
                        .withTimeout(ShooterConstants.shootAmpTime));

    }
}
