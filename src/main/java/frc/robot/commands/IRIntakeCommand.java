package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.LedConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedsSubsystem;

public class IRIntakeCommand extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final DigitalInput m_IRSensor;

    public IRIntakeCommand(IntakeSubsystem intakeSubsystem, DigitalInput irSensor) {
        m_IntakeSubsystem = intakeSubsystem;
        m_IRSensor = irSensor;
        addRequirements(m_IntakeSubsystem);
        SmartDashboard.putNumber("Intake Speed", Constants.IntakeConstants.DefaultSpeed);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_IntakeSubsystem
                .setIntakeSpeed(SmartDashboard.getNumber("Intake Speed", Constants.IntakeConstants.DefaultSpeed));

    }

    public boolean isFinished() {
        if (m_IRSensor.get()) {
            m_IntakeSubsystem.setIntakeSpeed(Constants.IntakeConstants.DefaultSpeed);
            return true;
        } else {
            return false;
        }
    }

}
