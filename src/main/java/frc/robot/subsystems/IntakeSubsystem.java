package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Utils.LogSubsystem;
import frc.robot.Utils.ShuffleBoardSubsystem;

public class IntakeSubsystem extends ShuffleBoardSubsystem {
    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;
    private final LogSubsystem m_logSubsystem;

    public IntakeSubsystem() {
        super();
        intakeMotor1 = new CANSparkMax(IntakeConstants.IntakeMotor1Id, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(IntakeConstants.IntakeMotor2Id, MotorType.kBrushless);

        m_logSubsystem = new LogSubsystem(this.getName());
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor1.set(speed);
        intakeMotor2.set(-speed);
        m_logSubsystem.logValue(speed);
    }
}
