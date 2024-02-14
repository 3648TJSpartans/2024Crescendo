package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utils.LogSubsystem;
import frc.robot.Utils.ShuffleBoardSubsystem;

public class ShooterSubsystem extends ShuffleBoardSubsystem {
    private final CANSparkMax m_shooterMotor1;
    private final CANSparkMax m_shooterMotor2;
    private final CANSparkMax m_beltMotor1;
    private final CANSparkMax m_beltMotor2;
    private final LogSubsystem m_logSubsystem;

    public ShooterSubsystem() {
        super();
        m_shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1Id, MotorType.kBrushless);
        m_shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2Id, MotorType.kBrushless);
        m_shooterMotor2.follow(m_shooterMotor1);
        m_shooterMotor1.setInverted(true);
        m_beltMotor1 = new CANSparkMax(ShooterConstants.beltMotorId1, MotorType.kBrushless);
        m_beltMotor2 = new CANSparkMax(ShooterConstants.beltMotorId2, MotorType.kBrushless);

        m_logSubsystem = new LogSubsystem(this.getName());
    }

    public void revShooter(double speed) {
        m_shooterMotor1.set(speed);
        m_logSubsystem.logValue(speed);
    }

    public void moveShooterIntake(double speed) {
        m_beltMotor1.set(speed);
        m_beltMotor2.set(speed);
        m_logSubsystem.logValue(speed);
    }

    public void moveBeltShuffleBoard() {
        double speed = SmartDashboard.getNumber("Belt Speed", 0);
        m_beltMotor1.set(speed);
        m_beltMotor2.set(speed);
        m_logSubsystem.logValue(speed);
    }

    public void moveShooterShuffleBoard() {
        double speed = SmartDashboard.getNumber("Shooter Speed", 0);
        m_shooterMotor1.set(speed);
        m_logSubsystem.logValue(speed);
    }
}
