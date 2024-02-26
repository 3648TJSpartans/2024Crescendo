package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                                new RevMotorCommand(m_shooterSubsystem, ShooterConstants.shooterSpeakerSpeed,
                                                ShooterConstants.shooterSpeakerSpeed)
                                                .withTimeout(ShooterConstants.revSpeakerTime),
                                // new WaitCommand(ShooterConstants.revSpeakerTime),
                                new ShootCommand(m_shooterSubsystem,
                                                ShooterConstants.shooterSpeakerSpeed,
                                                ShooterConstants.shooterSpeakerSpeed,
                                                ShooterConstants.beltSpeakerSpeed)
                                                .withTimeout(ShooterConstants.shootSpeakerTime),
                                new InstantCommand(() -> m_ledsSubsystem.setIntakeColor(m_IrSensor)));
        }

}
