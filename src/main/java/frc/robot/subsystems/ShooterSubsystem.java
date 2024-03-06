package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_shooterMotorTop;
    private final CANSparkMax m_shooterMotorBottom;
    private final CANSparkMax m_beltMotor1;
    private final SparkPIDController m_shooterMotorTopController;
    private final SparkPIDController m_shooterMotorBottomController;

    public ShooterSubsystem() {
        m_shooterMotorTop = new CANSparkMax(ShooterConstants.shooterMotorTopId, MotorType.kBrushless);
        m_shooterMotorBottom = new CANSparkMax(ShooterConstants.shooterMotorBottomId, MotorType.kBrushless);
        m_beltMotor1 = new CANSparkMax(ShooterConstants.beltMotorId1, MotorType.kBrushless);
        m_shooterMotorTopController = m_shooterMotorTop.getPIDController();
        m_shooterMotorBottomController = m_shooterMotorBottom.getPIDController();
        m_shooterMotorTopController.setFeedbackDevice(m_shooterMotorTop.getEncoder());
        m_shooterMotorBottomController.setFeedbackDevice(m_shooterMotorBottom.getEncoder());
        m_shooterMotorTopController.setP(ShooterConstants.kshooterP);
        m_shooterMotorBottomController.setP(ShooterConstants.kshooterP);
        m_shooterMotorTopController.setI(ShooterConstants.kshooterI);
        m_shooterMotorBottomController.setI(ShooterConstants.kshooterI);
        m_shooterMotorTopController.setD(ShooterConstants.kshooterD);
        m_shooterMotorBottomController.setD(ShooterConstants.kshooterD);
        m_shooterMotorTopController.setFF(ShooterConstants.kshooterFF);
        m_shooterMotorBottomController.setFF(ShooterConstants.kshooterFF);
        m_shooterMotorTop.setIdleMode(IdleMode.kCoast);
        m_shooterMotorBottom.setIdleMode(IdleMode.kCoast);
        m_beltMotor1.setIdleMode(IdleMode.kBrake);
        m_shooterMotorTop.burnFlash();
        m_shooterMotorBottom.burnFlash();

    }

    @Override
    public void periodic() {
    }

    public void setShooterVelocity(double topSpeed, double bottomSpeed) {
        m_shooterMotorTop.setInverted(true);
        m_shooterMotorBottom.setInverted(false);
        m_shooterMotorTopController.setReference(topSpeed, CANSparkMax.ControlType.kVelocity);
        m_shooterMotorBottomController.setReference(bottomSpeed, CANSparkMax.ControlType.kVelocity);

    }

    public void setInvertedShooterVelocity(double topSpeed, double bottomSpeed) {
        m_shooterMotorTop.setInverted(false);
        m_shooterMotorBottom.setInverted(true);
        m_shooterMotorTopController.setReference(topSpeed, CANSparkMax.ControlType.kVelocity);
        m_shooterMotorBottomController.setReference(bottomSpeed, CANSparkMax.ControlType.kVelocity);
    }

    public void setBeltSpeed(double speed) {
        m_beltMotor1.set(speed);
    }

}
