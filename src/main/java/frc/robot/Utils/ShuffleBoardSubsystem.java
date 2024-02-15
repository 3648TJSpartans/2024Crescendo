package frc.robot.Utils;

import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

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

    public void addSwerveMotorVals(SwerveModule mod, String s) {
        ShuffleboardLayout m_layout;
        // Loop through all the fields in the class
        // If the field is a CANSparkMax, add it to the shuffleboard
        try {
            Field[] fields = mod.getClass().getDeclaredFields();
            Field[] subsystemFields = this.getClass().getDeclaredFields();
            for (Field field : fields) {
                if (field.getType().isAssignableFrom(CANSparkMax.class)) {
                    // Get the CANSparkMax object from the field
                    field.setAccessible(true);
                    CANSparkMax sparkMax = (CANSparkMax) field.get(mod);
                    m_layout = Shuffleboard.getTab(this.getName()).getLayout(s,
                            BuiltInLayouts.kList).withSize(2, 3);
                    m_layout.add(field.getName() + "_Spd", sparkMax.getEncoder().getVelocity());
                    m_layout.add(field.getName() + "_Pos", sparkMax.getEncoder().getPosition());
                }
            }
            for (Field field : subsystemFields) {
                if (field.getType().isAssignableFrom(AHRS.class)) {
                    // Get the AHRS object from the field
                    field.setAccessible(true);
                    AHRS m_gyro = (AHRS) field.get(this);
                    Shuffleboard.getTab(this.getName()).add(field.getName(), m_gyro.getAngle()).withSize(1, 1);
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
