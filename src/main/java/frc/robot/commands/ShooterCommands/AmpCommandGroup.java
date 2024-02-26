package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpCommandGroup extends SequentialCommandGroup {
        private final ShooterSubsystem m_shooterSubsystem;
        private final LedsSubsystem m_ledsSubsystem;
        private final DigitalInput m_irSensor;

        public AmpCommandGroup(ShooterSubsystem shooterSubsystem, LedsSubsystem ledsSubsystem, DigitalInput irSensor) {
                m_shooterSubsystem = shooterSubsystem;
                m_ledsSubsystem = ledsSubsystem;
                m_irSensor = irSensor;
                addCommands(
                                new RevMotorCommand(m_shooterSubsystem, ShooterConstants.shooterAmpTopSpeed,
                                                ShooterConstants.shooterAmpBottomSpeed)
                                                .withTimeout(ShooterConstants.revAmpTime),
                                new ShootCommand(m_shooterSubsystem, ShooterConstants.shooterAmpTopSpeed,
                                                ShooterConstants.shooterAmpBottomSpeed, ShooterConstants.beltAmpSpeed)
                                                .withTimeout(ShooterConstants.shootAmpTime),
                                new InstantCommand(() -> m_ledsSubsystem.setIntakeColor(irSensor)));

        }
}
