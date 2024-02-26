package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommandGroup extends SequentialCommandGroup {
        private final ShooterSubsystem m_shooterSubsystem;

        public ShooterCommandGroup(ShooterSubsystem shooterSubsystem) {
                m_shooterSubsystem = shooterSubsystem;

                addCommands(
                                new RevMotorCommand(m_shooterSubsystem, ShooterConstants.shooterSpeakerSpeed,
                                                ShooterConstants.shooterSpeakerSpeed)
                                                .withTimeout(ShooterConstants.revSpeakerTime),
                                // new WaitCommand(ShooterConstants.revSpeakerTime),
                                new SequentialCommandGroup(
                                                new ShootCommand(m_shooterSubsystem,
                                                                ShooterConstants.shooterSpeakerSpeed,
                                                                ShooterConstants.shooterSpeakerSpeed,
                                                                ShooterConstants.beltSpeakerSpeed)
                                                                .withTimeout(ShooterConstants.shootSpeakerTime)));// Add
                                                                                                                  // Shooter
                                                                                                                  // Led's
                                                                                                                  // here
                                                                                                                  // in
                                                                                                                  // the
                                                                                                                  // sequential
                                                                                                                  // Command
                                                                                                                  // Group
                                                                                                                  // with
                                                                                                                  // the
                                                                                                                  // same
                                                                                                                  // timeout
                                                                                                                  // as
                                                                                                                  // shooterSpeakerTime
        }

}
