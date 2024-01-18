package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;
    private final CANSparkMax beltMotor;

    public ShooterSubsystem() {
        shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1Id, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2Id, MotorType.kBrushless);
        shooterMotor2.follow(shooterMotor1);
        shooterMotor1.setInverted(true);
        beltMotor = new CANSparkMax(ShooterConstants.beltMotorId, MotorType.kBrushless);
    }

    public void revShooter() {
        shooterMotor1.set(ShooterConstants.motorSpeed);
    }

    public void moveShooterIntake() {
        beltMotor.set(ShooterConstants.beltMotorSpeed);
    }

}