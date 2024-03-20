package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedsSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeLedCommand extends Command {
    private final LedsSubsystem m_ledsSubsystem;
    private final Timer m_ledTimer;
    private boolean m_activated;

    public IntakeLedCommand(LedsSubsystem ledsSubsystem, boolean activated) {
        m_ledsSubsystem = ledsSubsystem;
        m_activated = activated;
        m_ledTimer = new Timer();
        addRequirements(m_ledsSubsystem);
        m_ledsSubsystem.setColorHSV((m_activated ? 60 : 120), 1, 1, LedConstants.topBarLedStart,
                LedConstants.topBarLedEnd);
    }

    @Override
    public void initialize() {
        m_ledTimer.start();
    }

    @Override
    public void execute() {
        m_ledsSubsystem.intakeWave(5 * m_ledTimer.get(), (m_activated ? -2 : 2), 60, LedConstants.topBarLedStart,
                LedConstants.topBarLedQuat, true);
        m_ledsSubsystem.intakeWave(5 * m_ledTimer.get(), (m_activated ? -2 : 2), 60, LedConstants.topBarLedQuat,
                LedConstants.topBarLedHalf, false);
        m_ledsSubsystem.intakeWave(5 * m_ledTimer.get(), (m_activated ? -2 : 2), 60, LedConstants.topBarLedHalf,
                LedConstants.topBarLedThreeQuats, false);
        m_ledsSubsystem.intakeWave(5 * m_ledTimer.get(), (m_activated ? -2 : 2), 60, LedConstants.topBarLedThreeQuats,
                LedConstants.topBarLedEnd, true);
    }

    @Override
    public boolean isFinished() {
        if (m_ledTimer.get() > 14) {
            m_ledTimer.stop();
            m_ledTimer.reset();
            return true;
        } else {
            return false;
        }
    }
}
