package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Utils.LogSubsystem;
import frc.robot.Utils.ShuffleBoardSubsystem;

public class ClimberSubsystem extends ShuffleBoardSubsystem {
    private final CANSparkMax m_climberMotor1;
    private final CANSparkMax m_climberMotor2;
    private final SparkPIDController m_climberPIDController1;
    private final SparkPIDController m_climberPIDController2;
    private final RelativeEncoder m_climberEncoder1;
    private final RelativeEncoder m_climberEncoder2;
    private final LogSubsystem m_logSubsystem;

    public ClimberSubsystem() {
        super();
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

        m_logSubsystem = new LogSubsystem(this.getName());
    }

    public void setClimberPosition(double position) {
        m_climberPIDController1.setReference(position, CANSparkMax.ControlType.kPosition);
        m_climberPIDController2.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getClimberPosition() {
        double m_setPosition = m_climberEncoder1.getPosition();
        m_logSubsystem.logValue(m_setPosition);
        return m_setPosition;
    }

    public void MoveClimber(double speed) {
        m_climberMotor1.set(speed);
        m_climberMotor2.set(-speed);
        m_logSubsystem.logValue(speed);
    }

    public void updateShuffleBoard() {

    }
}