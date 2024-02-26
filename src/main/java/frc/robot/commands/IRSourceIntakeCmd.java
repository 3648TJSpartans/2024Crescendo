package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class IRSourceIntakeCmd extends Command {
    private final DigitalInput m_IRSensor;
    private final ShooterSubsystem m_shooterSubsystem;

    public IRSourceIntakeCmd(ShooterSubsystem shooterSubsystem,
            DigitalInput irSensor) {
        m_shooterSubsystem = shooterSubsystem;
        m_IRSensor = irSensor;
        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setShooterVelocity(-ShooterConstants.SourceShooterSpeed,
                -ShooterConstants.SourceShooterSpeed);
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.SourceBeltSpeed);

    }

    public boolean isFinished() {
        if (!m_IRSensor.get()) {
            m_shooterSubsystem.setShooterVelocity(ShooterConstants.DefaultSpeed, ShooterConstants.DefaultSpeed);
            m_shooterSubsystem.setBeltSpeed(ShooterConstants.DefaultSpeed);
            return true;
        } else {
            return false;
        }
    }

}
