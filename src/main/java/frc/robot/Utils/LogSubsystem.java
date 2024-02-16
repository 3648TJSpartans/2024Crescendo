package frc.robot.Utils;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LogSubsystem extends SubsystemBase {
    private final DoubleLogEntry m_EncoderLog;
    private final DataLog m_Log;

    public LogSubsystem(String subsystemName) {
        // Starts recording to data log
        DataLogManager.start();

        // Setup custom log entries
        m_Log = DataLogManager.getLog();
        m_EncoderLog = new DoubleLogEntry(m_Log, subsystemName);
    }

    public void logValue(double value) {
        m_EncoderLog.append(value);
    }
}
