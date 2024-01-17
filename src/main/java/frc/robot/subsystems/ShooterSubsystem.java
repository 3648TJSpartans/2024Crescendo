package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax ShooterMotor1;
    private final CANSparkMax ShooterMotor2;
    private final CANSparkMax BeltMotor;

    public ShooterSubsystem() {
        ShooterMotor1 = new CANSparkMax(ShooterConstants.ShooterMotor1Id, MotorType.kBrushless);
        ShooterMotor2 = new CANSparkMax(ShooterConstants.ShooterMotor2Id, MotorType.kBrushless);
        BeltMotor = new CANSparkMax(ShooterConstants. BeltMotorId, MotorType.kBrushless);
    }
    }

    public void revShooter() {
        ShooterMotor1.set(0.5);
        ShooterMotor2.set(-0.5);

    }

    public void moveShooterIntake() {
        BeltMotor.set(0.2);

    }

}
