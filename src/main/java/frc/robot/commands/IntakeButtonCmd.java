package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeButtonCmd extends Command {
    private final Supplier<Boolean> m_leftExecuteButton;
    private final Supplier<Boolean> m_rightExecuteButton;
    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeButtonCmd(IntakeSubsystem intakeSubsystem, Supplier<Boolean> leftExecuteButton,
            Supplier<Boolean> rightExecuteButton) {
        m_intakeSubsystem = intakeSubsystem;
        m_leftExecuteButton = leftExecuteButton;
        m_rightExecuteButton = rightExecuteButton;
        addRequirements(m_intakeSubsystem);
        SmartDashboard.putNumber("Intake Speed", 0);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        if (m_leftExecuteButton.get()) {
            m_intakeSubsystem.setIntakeSpeed(-SmartDashboard.getNumber("Intake Speed", 0));
        } else if (m_rightExecuteButton.get()) {
            m_intakeSubsystem.setIntakeSpeed(SmartDashboard.getNumber("Intake Speed", 0));
        } else {
            m_intakeSubsystem.setIntakeSpeed(0);
        }
    }
}
