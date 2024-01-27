package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;
  

    public IntakeSubsystem() {
        intakeMotor1 = new CANSparkMax(IntakeConstants.IntakeMotor1Id, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(IntakeConstants.IntakeMotor2Id, MotorType.kBrushless);
        
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor1.set(speed);
        intakeMotor2.set(-speed);
    }
}
