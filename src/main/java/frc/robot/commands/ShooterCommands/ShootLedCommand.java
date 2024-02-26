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
        // m_ledTimer.start();
        // m_ledsSubsystem.setColor(0, 127, 124, 0, lShoot);
        // if (ramp > 0.001) {
        // if (m_ledTimer.get() > ramp) {
        // lShoot += 1;
        // m_ledTimer.reset();
        // m_ledTimer.start();
        // if (lShoot > LedConstants.shooterLedEnd) {
        // lShoot = 0;
        // ramp /= 2;
        // m_ledsSubsystem.setColor(0, 0, 0, 0, LedConstants.shooterLedEnd);
        // }
        // }
        // } else {
        // if ((m_ledTimer.get() > 5) || (m_ledTimer.get() == 0)) {
        // m_ledsSubsystem.setColor(0, 0, 0, 0, LedConstants.shooterLedEnd);
        // m_ledTimer.stop();
        // m_ledTimer.reset();
        // ramp = 0.05;
        // } else {
        // m_ledsSubsystem.setColor(0, 127, 124, 0, LedConstants.shooterLedEnd);
        // }
        // lShoot = 0;
        // }
        if (m_ledTimer.get() < ramp) {
            m_ledsSubsystem.setColorRGB(0, 127, 174, LedConstants.shooterLedStart,
                    (int) (Math.round(m_ledTimer.get() * LedConstants.shooterLedEnd / ramp)));
        } else if (ramp > 0.001) {
            ramp /= 1.5;
            m_ledsSubsystem.setColorRGB(0, 0, 0, LedConstants.shooterLedStart, LedConstants.shooterLedEnd);
            m_ledTimer.reset();
        } else {
            if (m_ledTimer.get() < 3) {
                m_ledsSubsystem.setColorRGB(0, 127 - ((int) Math.round(m_ledTimer.get() * 127 / 3)),
                        174 - ((int) Math.round(m_ledTimer.get() * 174 / 3)), LedConstants.shooterLedStart,
                        LedConstants.shooterLedEnd);
            } else {
                m_ledsSubsystem.setColorRGB(0, 0, 0, LedConstants.shooterLedStart, LedConstants.shooterLedEnd);
            }
        }
        SmartDashboard.putNumber("_Ramp", ramp);
        SmartDashboard.putNumber("_Time", m_ledTimer.get());
    }

    @Override
    public boolean isFinished() {
        if (m_ledTimer.get() > 3 + 0.02) {
            m_ledTimer.stop();
            m_ledTimer.reset();
            ramp = 1;
            return true;
        } else {
            return false;
        }
    }
}
