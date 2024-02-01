package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Utils.ShuffleBoardSubsystem;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_climberMotor1;
    private final CANSparkMax m_climberMotor2;
    private final CANSparkMax[] m_motors;
    private final SparkPIDController m_climberPIDController1;
    private final SparkPIDController m_climberPIDController2;
    private final RelativeEncoder m_climberEncoder1;
    private final RelativeEncoder m_climberEncoder2;
    private final ShuffleBoardSubsystem m_shuffleBoardSubsystem;

    public ClimberSubsystem() {
        // Motor 1
        m_climberMotor1 = new CANSparkMax(ClimberConstants.climberMotor1ID, MotorType.kBrushless);
        m_climberEncoder1 = m_climberMotor1.getEncoder();
        m_climberPIDController1 = m_climberMotor1.getPIDController();
        m_climberPIDController1.setFeedbackDevice(m_climberEncoder1);
        m_climberPIDController1.setP(ClimberConstants.kClimberP);
        m_climberPIDController1.setI(ClimberConstants.kClimberI);
        m_climberPIDController1.setD(ClimberConstants.kClimberD);
        m_climberPIDController1.setFF(ClimberConstants.kClimberFF);
        m_climberPIDController1.setOutputRange(ClimberConstants.kClimberMinOutput, ClimberConstants.kClimberMaxOutput);
        // Motor 2
        m_climberMotor2 = new CANSparkMax(ClimberConstants.climberMotor2ID, MotorType.kBrushless);
        m_climberEncoder2 = m_climberMotor2.getEncoder();
        m_climberPIDController2 = m_climberMotor2.getPIDController();
        m_climberPIDController2.setFeedbackDevice(m_climberEncoder2);
        m_climberPIDController2.setP(ClimberConstants.kClimberP);
        m_climberPIDController2.setI(ClimberConstants.kClimberI);
        m_climberPIDController2.setD(ClimberConstants.kClimberD);
        m_climberPIDController2.setFF(ClimberConstants.kClimberFF);
        m_climberPIDController2.setOutputRange(ClimberConstants.kClimberMinOutput, ClimberConstants.kClimberMaxOutput);

        m_motors = new CANSparkMax[] { m_climberMotor1, m_climberMotor2 };
        m_shuffleBoardSubsystem = new ShuffleBoardSubsystem(this.getName());
        m_shuffleBoardSubsystem.addVals(this.getName(), m_motors);
    }

    public void setClimberPosition(double position) {
        m_climberPIDController1.setReference(position, CANSparkMax.ControlType.kPosition);
        m_climberPIDController2.setReference(position, CANSparkMax.ControlType.kPosition);

    }

    public double getClimberPosition() {
        double m_setPosition = m_climberEncoder1.getPosition();
        return m_setPosition;
    }

    public void MoveClimber(double speed) {
        m_climberMotor1.set(speed);
        m_climberMotor2.set(-speed);
    }
}