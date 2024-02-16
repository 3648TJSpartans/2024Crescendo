package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ArmConstants;
import frc.robot.Utils.ShuffleBoardSubsystem;

public class ArmSubsystem extends ShuffleBoardSubsystem {
    private final CANSparkMax armMotor;

    public ArmSubsystem() {
        super();
        armMotor = new CANSparkMax(ArmConstants.armMotorId, MotorType.kBrushless);
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
