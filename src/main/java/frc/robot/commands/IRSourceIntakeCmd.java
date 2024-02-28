package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IRSourceIntakeCmd extends Command {
    private final DigitalInput m_IRSensor;
    private final ShooterSubsystem m_shooterSubsystem;
    private final LedsSubsystem m_ledSubsystem;

    public IRSourceIntakeCmd(ShooterSubsystem shooterSubsystem, LedsSubsystem ledsSubsystem,
            DigitalInput irSensor) {
        m_shooterSubsystem = shooterSubsystem;
        m_IRSensor = irSensor;
        m_ledSubsystem = ledsSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_ledSubsystem.setColor(LedConstants.sourceRunningRGB, LedConstants.topBarLedStart, LedConstants.topBarLedStop);

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setInvertedShooterVelocity(ShooterConstants.SourceShooterSpeed,
                ShooterConstants.SourceShooterSpeed);
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.SourceBeltSpeed);

    }

    public boolean isFinished() {
        if (!m_IRSensor.get()) {
            m_shooterSubsystem.setBeltSpeed(ShooterConstants.SourceBeltSpeed);
            if (m_IRSensor.get()) {
                m_ledSubsystem.setColor(LedConstants.yesNoteRGB, LedConstants.topBarLedStart,
                        LedConstants.topBarLedStop);
                m_shooterSubsystem.setShooterVelocity(ShooterConstants.DefaultSpeed, ShooterConstants.DefaultSpeed);
                m_shooterSubsystem.setBeltSpeed(ShooterConstants.DefaultSpeed);
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

}
