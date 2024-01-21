package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;

public class TrapSubsystem extends SubsystemBase {
    private final CANSparkMax m_trapMotor_UpDown;
    private final CANSparkMax m_trapMotor_InOut;
    private final CANSparkMax m_trapMotor_Track;
    private static AbsoluteEncoder m_trapEncoderAbsolute;
    private static RelativeEncoder m_trapEncoderRelative;
    private final SparkPIDController m_UpDown_PIDController;

    // "m_trapMotor_InOut" based off ArmSubsystem.java 
    // "m_trapMotor_UpDown" based off ClimberSubsystem.java
    public TrapSubsystem(){
        // Up & Down Motor
        m_trapMotor_UpDown = new CANSparkMax(0, MotorType.kBrushless);
        m_trapEncoderRelative = m_trapMotor_UpDown.getEncoder();        
        m_UpDown_PIDController = m_trapMotor_UpDown.getPIDController();
        m_UpDown_PIDController.setFeedbackDevice(m_trapEncoderRelative);
        m_UpDown_PIDController.setP(TrapConstants.kTrapP);
        m_UpDown_PIDController.setI(TrapConstants.kTrapI);
        m_UpDown_PIDController.setD(TrapConstants.kTrapD);
        m_UpDown_PIDController.setFF(TrapConstants.kTrapFF);
        m_UpDown_PIDController.setOutputRange(TrapConstants.kTrapMinOutput, TrapConstants.kTrapMaxOutput);
        
        // In & Out Motor
        m_trapMotor_InOut = new CANSparkMax(0, MotorType.kBrushless);
        m_trapEncoderAbsolute = m_trapMotor_InOut.getAbsoluteEncoder(Type.kDutyCycle);  
        
        // Track Motor
        m_trapMotor_Track = new CANSparkMax(0, MotorType.kBrushless);
    }

    public void setUpDownPosition(double position){
        m_UpDown_PIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public static double getUpDownPosition(){
        double m_setPosition = m_trapEncoderRelative.getPosition();
        return m_setPosition;
    }

    public void moveUpDown(double speed){
        m_trapMotor_UpDown.set(speed);
    }

    public static double getInOutAngle(){
        double m_InOutAngle = m_trapEncoderAbsolute.getPosition();
        return m_InOutAngle;
    }

    public void moveInOut(double speed){
        m_trapMotor_InOut.set(speed); 
    }
    
    public void moveTrack(double speed){
        m_trapMotor_Track.set(speed);
    }
}