package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommandGroup extends SequentialCommandGroup {
        private final ShooterSubsystem m_shooterSubsystem;
        private final LedsSubsystem m_ledsSubsystem;
        private final DigitalInput m_IrSensor;

        public ShooterCommandGroup(ShooterSubsystem shooterSubsystem, LedsSubsystem ledsSubsystem,
                        DigitalInput IrSensor) {
                m_shooterSubsystem = shooterSubsystem;
                m_ledsSubsystem = ledsSubsystem;
                m_IrSensor = IrSensor;

                addCommands(
                                new SequentialCommandGroup(new RevMotorCommand(m_shooterSubsystem,
                                                ShooterConstants.shooterTopSpeakerSpeed,
                                                ShooterConstants.shooterSpeakerBottomSpeed),
                                                new InstantCommand(() -> m_ledsSubsystem.setColor(LedConstants.revRGB,
                                                                LedConstants.topBarLedStart,
                                                                LedConstants.topBarLedStop)))
                                                .withTimeout(ShooterConstants.revSpeakerTime),
                                new SequentialCommandGroup(
                                                new ShootCommand(m_shooterSubsystem,
                                                                ShooterConstants.shooterTopSpeakerSpeed,
                                                                ShooterConstants.shooterSpeakerBottomSpeed,
                                                                ShooterConstants.beltSpeakerSpeed),
                                                new InstantCommand(() -> m_ledsSubsystem.setColor(LedConstants.shootRGB,
                                                                LedConstants.topBarLedStart,
                                                                LedConstants.topBarLedStop)))
                                                .withTimeout(ShooterConstants.shootSpeakerTime),
                                new InstantCommand(() -> m_ledsSubsystem.setIntakeColor(m_IrSensor)));
        }

}
