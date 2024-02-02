package frc.robot.Utils;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.List;
import java.util.Map;
import java.util.Arrays;

public class ShuffleBoardSubsystem {
    private final Map<Integer, String> motorIDs = Map.ofEntries(
            Map.entry(1, "FL-Turning"),
            Map.entry(2, "FL Driving"),
            Map.entry(3, "FR-Turning"),
            Map.entry(4, "FR-Driving"),
            Map.entry(5, "RR-Turning"),
            Map.entry(6, "RR-Driving"),
            Map.entry(7, "RL-Turning"),
            Map.entry(8, "RL-Driving"),
            Map.entry(9, "Intake 1"),
            Map.entry(10, "Intake 2"),
            Map.entry(11, "Climber 1"),
            Map.entry(12, "Climber 2"),
            Map.entry(13, "Belt 1"),
            Map.entry(14, "Shooter 1"),
            Map.entry(15, "Belt 2"),
            Map.entry(16, "Shooter 2"));

    private final String m_subsystemName;
    private final ShuffleboardTab m_tab;
    private ShuffleboardLayout m_layout;

    public ShuffleBoardSubsystem(String subsystemName) {
        m_subsystemName = subsystemName;
        m_tab = Shuffleboard.getTab(m_subsystemName);
    }

    public void addVals(String tabName, CANSparkMax[] motors) {

        for (CANSparkMax motor : motors) {
            m_layout = Shuffleboard.getTab(tabName).getLayout(motorIDs.get(motor.getDeviceId()), BuiltInLayouts.kList)
                    .withSize(2, 2);
            m_layout.add(motorIDs.get(motor.getDeviceId()) + " Spd", motor.getEncoder().getVelocity());
            m_layout.add(motorIDs.get(motor.getDeviceId()) + " Pos", motor.getEncoder().getPosition());
        }
    }
}
