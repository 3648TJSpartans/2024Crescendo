package frc.robot.Utils;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.List;
import java.util.Map;
import java.util.Arrays;

public class ShuffleBoardSubsystem {
    private final Map<Integer, String> motorIDs = Map.ofEntries(
            Map.entry(1, "Front Left Turning"),
            Map.entry(2, "Front Left Driving"),
            Map.entry(3, "Front Right Turning"),
            Map.entry(4, "Front Right Driving"),
            Map.entry(5, "Rear Right Turning"),
            Map.entry(6, "Rear Right Driving"),
            Map.entry(7, "Rear Left Turning"),
            Map.entry(8, "Rear Left Driving"),
            Map.entry(9, "Intake Motor 1"),
            Map.entry(10, "Intake Motor 2"),
            Map.entry(11, "Climber Motor 1"),
            Map.entry(12, "Climber Motor 2"),
            Map.entry(13, "Belt Motor 1"),
            Map.entry(14, "Shooter Motor 1"),
            Map.entry(15, "Belt Motor 2"),
            Map.entry(16, "Shooter Motor 2"));

    private final String m_subsystemName;
    private final ShuffleboardTab m_tab;

    public ShuffleBoardSubsystem(String subsystemName) {
        m_subsystemName = subsystemName;
        m_tab = Shuffleboard.getTab(m_subsystemName);
    }

    public void addVals(String tabName, CANSparkMax[] motors) {
        for (CANSparkMax motor : motors) {
            Shuffleboard.getTab(tabName).add(motorIDs.get(motor.getDeviceId()), motor.get());
        }
    }
}
