package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Utils.LogSubsystem;
import frc.robot.Utils.ShuffleBoardSubsystem;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;
    private final CANSparkMax[] m_motors;
    private final ShuffleBoardSubsystem m_shuffleBoardSubsystem;
    private final LogSubsystem m_logSubsystem;

    public IntakeSubsystem() {
        intakeMotor1 = new CANSparkMax(IntakeConstants.IntakeMotor1Id, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(IntakeConstants.IntakeMotor2Id, MotorType.kBrushless);
        m_motors = new CANSparkMax[] { intakeMotor1, intakeMotor2 };
        m_shuffleBoardSubsystem = new ShuffleBoardSubsystem(this.getName());
        m_shuffleBoardSubsystem.addVals(this.getName(), m_motors);

        m_logSubsystem = new LogSubsystem(this.getName());
    }

    public void setIntakeSpeed(double speed) {

        intakeMotor1.set(speed);
        intakeMotor2.set(-speed);
        m_logSubsystem.logValue(speed);
    }
}
