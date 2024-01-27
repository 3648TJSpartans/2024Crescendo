package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends PIDSubsystem {
    private final CANSparkMax m_climberMotor1;
    private final CANSparkMax m_climberMotor2;
    private final SparkPIDController m_climberPIDController1;
    private final SparkPIDController m_climberPIDController2;
    private final RelativeEncoder m_climberEncoder1;
    private final RelativeEncoder m_climberEncoder2;

    //For the PID Subsystem
    private final Encoder m_cEncoder1 = new Encoder(0, 0, 0);
    private final Encoder m_cEncoder2 = new Encoder(0, 0, 0);
    private final SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(0, 0);

    public ClimberSubsystem() {
        super(new PIDController(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD));
        setSetpoint(ClimberConstants.kClimberHeight);
        // The zeros are magic numbers that are placeholders until more info is obtained about "setDistancePerPulse"
        m_cEncoder1.setDistancePerPulse(0);
        m_cEncoder2.setDistancePerPulse(0);

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


    // These two methods are created for the PID subsystem; WIll continue research on these methods.
    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        double Voltage = output + m_Feedforward.calculate(setpoint);
        m_climberMotor1.setVoltage(Voltage);
        m_climberMotor2.setVoltage(Voltage);
        throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
        return m_controller.getRate();
    }
}