package frc.robot.Utils;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.subsystems.Swerve.SwerveModule;

import java.lang.reflect.*;

public class ShuffleBoardSubsystem extends SubsystemBase {
    private final ShuffleboardTab m_tab;

    public ShuffleBoardSubsystem() {
        m_tab = Shuffleboard.getTab(this.getName());
    }

    public void addSwerveMotorVals(SwerveModule mod, int index) {
        ShuffleboardLayout m_layout;
        // Loop through all the fields in the class
        // If the field is a CANSparkMax, add it to the shuffleboard
        try {
            Field[] fields = mod.getClass().getDeclaredFields();
            for (Field field : fields) {
                if (field.getType().isAssignableFrom(CANSparkMax.class)) {
                    // Get the CANSparkMax object from the field
                    field.setAccessible(true);
                    CANSparkMax sparkMax = (CANSparkMax) field.get(mod);
                    m_layout = Shuffleboard.getTab("SwerveSubsystem").getLayout("module" + index,
                            BuiltInLayouts.kList).withSize(2, 3);
                    if (field.getName() == "m_drivingSparkMax") {
                        m_layout.add("module" + index + "Driving_Spd", sparkMax.getEncoder().getVelocity());
                        m_layout.add("module" + index + "Driving_Pos", sparkMax.getEncoder().getPosition());
                    } else if (field.getName() == "m_turningSparkMax") {
                        m_layout.add("module" + index + "Turning_Spd", sparkMax.getEncoder().getVelocity());
                        m_layout.add("module" + index + "Turning_Pos", sparkMax.getEncoder().getPosition());
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
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
                    field.setAccessible(true);
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
