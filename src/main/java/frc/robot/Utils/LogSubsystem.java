package frc.robot.Utils;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LogSubsystem extends SubsystemBase {
    DoubleLogEntry m_EncoderLog;

    public LogSubsystem(String subsystemName) {
    }

    public double logValue(double value) {
        m_EncoderLog.append(value);
        return value;
    }
}
