package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor;
    public ArmSubsystem() {
             armMotor = new CANSparkMax(armMotorId, MotorType.kBrushless);
    }

    public void setAngle(double angle) {

    }

    public double getAngle() {
        return 12.4; // TODO: change
    }

    public void MoveArm(double power) {

        armMotor.set(power);

    }

    @Override
    public void periodic() {

    }
}
