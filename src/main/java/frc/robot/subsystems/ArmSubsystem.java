package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utils.ShuffleBoardSubsystem;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor;
    private final ShuffleBoardSubsystem m_shuffleBoardSubsystem;

    public ArmSubsystem() {
        armMotor = new CANSparkMax(ArmConstants.armMotorId, MotorType.kBrushless);
        m_shuffleBoardSubsystem = new ShuffleBoardSubsystem(this.getName());
        m_shuffleBoardSubsystem.addValsbyClass(this.getName(), this.getClass());
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
