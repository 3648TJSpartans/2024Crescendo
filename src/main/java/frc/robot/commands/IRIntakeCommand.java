package frc.robot.commands;

import org.ejml.interfaces.decomposition.LUSparseDecomposition;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IRIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final DigitalInput m_IRSensor;
    private final ShooterSubsystem m_shooterSubsystem;

    public IRIntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, DigitalInput irSensor) {
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_IRSensor = irSensor;
        addRequirements(m_intakeSubsystem);
        addRequirements(m_shooterSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.beltAmpSpeed);
        m_intakeSubsystem
                .setIntakeSpeed(IntakeConstants.IntakeSpeed);

    }

    public boolean isFinished() {
        if (!m_IRSensor.get()) {

            m_intakeSubsystem.setIntakeSpeed(Constants.IntakeConstants.DefaultSpeed);
            m_shooterSubsystem.setBeltSpeed(ShooterConstants.DefaultSpeed);
            m_shooterSubsystem.setShooterVelocity(ShooterConstants.DefaultSpeed, ShooterConstants.DefaultSpeed);
            return true;
        } else {
            return false;
        }
    }

}
