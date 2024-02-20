package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.Constants.IRSensorConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_shooterMotor1;
    private final CANSparkMax m_shooterMotor2;
    private final CANSparkMax m_beltMotor1;

    public ShooterSubsystem() {
        m_shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1Id, MotorType.kBrushless);
        m_shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2Id, MotorType.kBrushless);
        m_beltMotor1 = new CANSparkMax(ShooterConstants.beltMotorId1, MotorType.kBrushless);
        m_shooterMotor1.setIdleMode(IdleMode.kCoast);
        m_shooterMotor2.setIdleMode(IdleMode.kCoast);
        SmartDashboard.putNumber("Belt Speed", 0);
        SmartDashboard.putNumber("Shooter Speed Top", 0);
        SmartDashboard.putNumber("Shooter Speed Bottom", 0);


    }

    @Override
    public void periodic() {
         SmartDashboard.putNumber("Shooter Top Velocity",m_shooterMotor1.getEncoder().getVelocity());
          SmartDashboard.putNumber("Shooter Bottom Velocity",m_shooterMotor2.getEncoder().getVelocity());
    }


    public void revShooter(double speed1, double speed2) {
        m_shooterMotor1.set(-speed1);
        m_shooterMotor2.set(speed2);

    }

    public void shuffleboardShooter() {
        m_shooterMotor1.set(-SmartDashboard.getNumber("Shooter Speed Top", 0));
        m_shooterMotor2.set(SmartDashboard.getNumber("Shooter Speed Bottom", 0));


    }

    public void shuffleboardBelts() {
        m_beltMotor1.set(SmartDashboard.getNumber("Belt Speed", 0));
    }

    public void moveShooterIntake(double speed) {
        m_beltMotor1.set(speed);
    }

}
