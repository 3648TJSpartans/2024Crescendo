package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedsSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootLedCommand extends Command {
    private final LedsSubsystem m_ledsSubsystem;

    private final Timer m_ledTimer;
    private double ramp = 1;
    // private int lShoot = 0;

    public ShootLedCommand(LedsSubsystem ledsSubsystem) {
        m_ledsSubsystem = ledsSubsystem;
        m_ledTimer = new Timer();
        addRequirements(m_ledsSubsystem);
        // m_ledTimer.start();
    }

    @Override
    public void initialize() {
        m_ledTimer.start();
    }

    @Override
    public void execute() {
        if (m_ledTimer.get() < ramp) {
            m_ledsSubsystem.pewPewWave(m_ledTimer.get() / ramp, 255, LedConstants.shooterLedStart,
                    LedConstants.shooterLedMiddleDiv, false);
            m_ledsSubsystem.pewPewWave(m_ledTimer.get() / ramp, 255, LedConstants.shooterLedMiddleDiv,
                    LedConstants.shooterLedEnd, true);
        } else if (ramp > 0.001) {
            ramp /= 1.5;
            m_ledsSubsystem.pewPewWave(0, 255, LedConstants.shooterLedStart, LedConstants.shooterLedEnd, false);
            m_ledTimer.reset();
        } else {
            if (m_ledTimer.get() < 2) {
                m_ledsSubsystem.pewPewWave(0, (int) Math.round(255 * m_ledTimer.get() / 2), LedConstants.shooterLedStart, LedConstants.shooterLedEnd, false);
            } else {
                m_ledsSubsystem.pewPewWave(0, 255, LedConstants.shooterLedStart, LedConstants.shooterLedEnd, false);
            }
        }
        // m_ledsSubsystem.pewPewWave(m_ledTimer.get() / 2,
        // LedConstants.shooterLedStart,
        // LedConstants.shooterLedMiddleDiv, false);
        // m_ledsSubsystem.pewPewWave(m_ledTimer.get() / 2,
        // LedConstants.shooterLedMiddleDiv, LedConstants.shooterLedEnd, true);
        SmartDashboard.putNumber("_Ramp", ramp);
        SmartDashboard.putNumber("_Time", m_ledTimer.get());
    }

    @Override
    public boolean isFinished() {
        if (m_ledTimer.get() > 2 + 0.02) {
            m_ledTimer.stop();
            m_ledTimer.reset();
            ramp = 1;
            return true;
        } else {
            return false;
        }
    }
}
