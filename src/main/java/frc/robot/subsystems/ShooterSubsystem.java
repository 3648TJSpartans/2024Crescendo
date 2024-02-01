package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utils.ShuffleBoardSubsystem;

import java.util.ArrayList;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_shooterMotor1;
    private final CANSparkMax m_shooterMotor2;
    private final CANSparkMax m_beltMotor1;
    private final CANSparkMax m_beltMotor2;
    private final CANSparkMax[] m_motors;
    private final ShuffleBoardSubsystem m_shuffleBoardSubsystem;

    public ShooterSubsystem() {
        m_shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1Id, MotorType.kBrushless);
        m_shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2Id, MotorType.kBrushless);
        m_beltMotor1 = new CANSparkMax(ShooterConstants.beltMotorId1, MotorType.kBrushless);
        m_beltMotor2 = new CANSparkMax(ShooterConstants.beltMotorId2, MotorType.kBrushless);
        m_motors = new CANSparkMax[] { m_beltMotor1, m_beltMotor2, m_shooterMotor1, m_shooterMotor2 };
        m_shuffleBoardSubsystem = new ShuffleBoardSubsystem(this.getName());
        m_shuffleBoardSubsystem.addVals(this.getName(), m_motors);
    }

    public void revShooter(double speed) {
        m_shooterMotor1.set(-speed);
        m_shooterMotor2.set(speed);

    }

    public void moveShooterIntake(double speed) {
        m_beltMotor1.set(-speed);
        m_beltMotor2.set(speed);
    }

    public void shuffleboardShooter() {
        m_shooterMotor1.set(-SmartDashboard.getNumber("Shooter Speed", 0));
        m_shooterMotor2.set(SmartDashboard.getNumber("Shooter Speed", 0));

    }

    public void shuffleboardBelts() {
        m_beltMotor1.set(-SmartDashboard.getNumber("Belt Speed", 0));
        m_beltMotor2.set(SmartDashboard.getNumber("Belt Speed", 0));
    }

}
