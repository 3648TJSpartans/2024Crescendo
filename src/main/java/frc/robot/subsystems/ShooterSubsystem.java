package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_shooterMotor1;
    private final CANSparkMax m_shooterMotor2;
    private final CANSparkMax m_beltMotor1;
    private final CANSparkMax m_beltMotor2;

    public ShooterSubsystem() {
        m_shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1Id, MotorType.kBrushless);
        m_shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2Id, MotorType.kBrushless);
        m_shooterMotor2.follow(m_shooterMotor1);
        m_shooterMotor1.setInverted(true);
        m_beltMotor1 = new CANSparkMax(ShooterConstants.beltMotorId1, MotorType.kBrushless);
        m_beltMotor2 = new CANSparkMax(ShooterConstants.beltMotorId2, MotorType.kBrushless);

    }

    public void revShooter(double speed) {
        m_shooterMotor1.set(speed);

    }

    public void moveShooterIntake(double speed) {
        m_beltMotor1.set(speed);
        m_beltMotor2.set(speed);
    }
}
