package frc.robot.Utils;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.lang.reflect.*;

public class ShuffleBoardSubsystem extends SubsystemBase {
    private final ShuffleboardTab m_tab;

    public ShuffleBoardSubsystem() {
        m_tab = Shuffleboard.getTab(this.getName());
    }

    @Override
    public void periodic() {
        ShuffleboardLayout m_layout;
        // Loop through all the fields in the class
        // If the field is a CANSparkMax, add it to the shuffleboard
        try {
            Field[] fields = this.getClass().getDeclaredFields();
            for (Field field : fields) {
                if (field.getType().isAssignableFrom(CANSparkMax.class)) {
                    // Get the CANSparkMax object from the field
                    CANSparkMax sparkMax = (CANSparkMax) field.get(this);
                    m_layout = Shuffleboard.getTab(this.getName()).getLayout(field.getName(),
                            BuiltInLayouts.kList).withSize(2, 2);
                    m_layout.add(field.getName() + "_Spd", sparkMax.getEncoder().getVelocity());
                    m_layout.add(field.getName() + "_Pos", sparkMax.getEncoder().getPosition());
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
