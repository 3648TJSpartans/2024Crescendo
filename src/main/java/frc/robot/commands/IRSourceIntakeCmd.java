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
    private boolean m_passIrSensorDown2;
    private boolean m_brokenIrSensorDown1;
    private boolean m_isFinished;

    public IRSourceIntakeCmd(ShooterSubsystem shooterSubsystem, LedsSubsystem ledsSubsystem,
            DigitalInput irSensor) {
        m_shooterSubsystem = shooterSubsystem;
        m_IRSensor = irSensor;
        m_ledSubsystem = ledsSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_isFinished = false;
        m_brokenIrSensorDown1 = false;
        m_passIrSensorDown2 = false;
        m_ledSubsystem.setColor(LedConstants.sourceRunningRGB, LedConstants.topBarLedStart, LedConstants.topBarLedStop);
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.SourceBeltSpeed);
        m_shooterSubsystem.setInvertedShooterVelocity(ShooterConstants.SourceShooterSpeed,
                ShooterConstants.SourceShooterSpeed);

    }

    @Override
    public void execute() {
        System.out.println("isFinished: " + m_isFinished);
        if (!m_brokenIrSensorDown1) {
            if (!m_IRSensor.get()) {
                m_brokenIrSensorDown1 = true;

            }
            return;
            // return false;
        }
        if (!m_passIrSensorDown2) {
            if (m_IRSensor.get()) {
                m_passIrSensorDown2 = true;
            }
            return;
            // return false;

        }
        m_shooterSubsystem.setBeltSpeed(-ShooterConstants.SourceBeltSpeed);
        if (m_IRSensor.get()) {
            // return false;
            return;
        } else {
            // return true;
            m_isFinished = true;
        }

    }

    public boolean isFinished() {
        return m_isFinished;

    }

    @Override
    public void end(boolean interrupted) {
        m_ledSubsystem.setColor(LedConstants.yesNoteRGB, LedConstants.topBarLedStart,
                LedConstants.topBarLedStop);
        m_shooterSubsystem.setShooterVelocity(ShooterConstants.DefaultSpeed, ShooterConstants.DefaultSpeed);
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.DefaultSpeed);

    }

}
