package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;
import frc.robot.Utils.LogSubsystem;
import frc.robot.Utils.ShuffleBoardSubsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;

public class TrapSubsystem extends SubsystemBase {
    private final CANSparkMax m_trapMotorUpDown;
    private final CANSparkMax m_trapMotorInOut;
    private final CANSparkMax m_trapMotorTrack;
    private static AbsoluteEncoder m_trapEncoderAbsolute;
    private static RelativeEncoder m_trapEncoderRelative;
    private final SparkPIDController m_UpDownPIDController;
    private final ShuffleBoardSubsystem m_shuffleBoardSubsystem;
    private static LogSubsystem m_logSubsystem;

    // "m_trapMotorInOut" based off ArmSubsystem.java
    // "m_trapMotorUpDown" based off ClimberSubsystem.java
    public TrapSubsystem() {
        // Up & Down Motor
        m_trapMotorUpDown = new CANSparkMax(TrapConstants.kUpDownMotorId, MotorType.kBrushless);
        m_trapEncoderRelative = m_trapMotorUpDown.getEncoder();
        m_UpDownPIDController = m_trapMotorUpDown.getPIDController();
        m_UpDownPIDController.setFeedbackDevice(m_trapEncoderRelative);
        m_UpDownPIDController.setP(TrapConstants.kTrapP);
        m_UpDownPIDController.setI(TrapConstants.kTrapI);
        m_UpDownPIDController.setD(TrapConstants.kTrapD);
        m_UpDownPIDController.setFF(TrapConstants.kTrapFF);
        m_UpDownPIDController.setOutputRange(TrapConstants.kTrapMinOutput, TrapConstants.kTrapMaxOutput);

        // In & Out Motor
        m_trapMotorInOut = new CANSparkMax(TrapConstants.kInOutMotorId, MotorType.kBrushless);
        m_trapEncoderAbsolute = m_trapMotorInOut.getAbsoluteEncoder(Type.kDutyCycle);

        // Track Motor
        m_trapMotorTrack = new CANSparkMax(TrapConstants.kTrackMotorId, MotorType.kBrushless);

        m_shuffleBoardSubsystem = new ShuffleBoardSubsystem(this.getName());
        m_shuffleBoardSubsystem.addValsbyClass(this.getName(), this.getClass());

        m_logSubsystem = new LogSubsystem(this.getName());
    }

    public void setUpDownPosition(double position) {
        m_UpDownPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public static double getUpDownPosition() {
        double m_setPosition = m_trapEncoderRelative.getPosition();
        m_logSubsystem.logValue(m_setPosition);
        return m_setPosition;
    }

    public void moveUpDown(double speed) {
        m_trapMotorUpDown.set(speed);
        m_logSubsystem.logValue(speed);
    }

    public static double getInOutAngle() {
        double m_InOutAngle = m_trapEncoderAbsolute.getPosition();
        m_logSubsystem.logValue(m_InOutAngle);
        return m_InOutAngle;
    }

    public void moveInOut(double speed) {
        m_trapMotorInOut.set(speed);
        m_logSubsystem.logValue(speed);
    }

    public void moveTrack(double speed) {
        m_trapMotorTrack.set(speed);
        m_logSubsystem.logValue(speed);
    }
}