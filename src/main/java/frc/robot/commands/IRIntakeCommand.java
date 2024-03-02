package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.ejml.interfaces.decomposition.LUSparseDecomposition;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IRIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final DigitalInput m_IRSensor;
    private final ShooterSubsystem m_shooterSubsystem;
    private final LedsSubsystem m_ledsSubsystem;
    private Boolean isFinished;

    public IRIntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, DigitalInput irSensor,
            LedsSubsystem ledsSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_ledsSubsystem = ledsSubsystem;
        m_IRSensor = irSensor;
        addRequirements(m_intakeSubsystem);
        addRequirements(m_shooterSubsystem);
        addRequirements(m_ledsSubsystem);

    }

    @Override
    public void initialize() {
        m_ledsSubsystem.setColor(LedConstants.intakeRunningRGB, LedConstants.topBarLedStart,
                LedConstants.topBarLedStop);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.beltAmpSpeed);
        m_intakeSubsystem
                .setIntakeSpeed(IntakeConstants.IntakeSpeed);

    }

    public boolean isFinished() {
        if (!m_IRSensor.get()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ledsSubsystem.setIntakeColor(m_IRSensor);
        m_intakeSubsystem.setIntakeSpeed(Constants.IntakeConstants.DefaultSpeed);
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.DefaultSpeed);
        m_shooterSubsystem.setShooterVelocity(ShooterConstants.DefaultSpeed, ShooterConstants.DefaultSpeed);
    }
    // {
    // m_intakeSubsystem.setIntakeSpeed(IntakeConstants.DefaultSpeed);
    // m_shooterSubsystem.setBeltSpeed(ShooterConstants.DefaultSpeed);
    // m_ledsSubsystem.setIntakeColor(m_IRSensor);
    // }

}
