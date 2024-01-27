package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IRSensorConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_shooterMotor1;
    private final CANSparkMax m_shooterMotor2;
    private final CANSparkMax m_beltMotor;
    private final DigitalInput IRSensor;

    public ShooterSubsystem() {
        m_shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1Id, MotorType.kBrushless);
        m_shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2Id, MotorType.kBrushless);
        m_shooterMotor2.follow(m_shooterMotor1);
        m_shooterMotor1.setInverted(true);
        m_beltMotor = new CANSparkMax(ShooterConstants.beltMotorId, MotorType.kBrushless);
        IRSensor = new DigitalInput(Constants.IRSensorConstants.IRSensorID);
    }

    public void revShooter(double speed) {
        m_shooterMotor1.set(speed);

    }

    public boolean NoteLocation() {
        return IRSensor.get();
    }

    public void periodic() {
        SmartDashboard.putBoolean("Ir Sensor", IRSensor.get());

    }

    public void moveShooterIntake(double speed) {
        m_beltMotor.set(speed);
    }

}
