package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_climberMotor;
    private final AbsoluteEncoder m_climberEncoder;
    private final SparkPIDController m_climberPIDController;

    public ClimberSubsystem() {
        m_climberMotor = new CANSparkMax(ArmConstants.armMotorId, MotorType.kBrushless);
        m_climberEncoder = m_climberMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_climberPIDController = m_climberMotor.getPIDController();
        m_climberPIDController.setFeedbackDevice(m_climberEncoder);
    }

    public void setClimberPosition(double position) {

    }

    public double getClimberPosition() {
        return 5.5;
    }

    public void MoveClmiber(double speed) {
        m_climberMotor.set(speed);
    }

}
