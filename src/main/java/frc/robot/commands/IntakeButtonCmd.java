package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeButtonCmd extends Command {
    private final Supplier<Boolean> m_leftexecuteButton;
    private final Supplier<Boolean> m_rightexecuteButton;
    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeButtonCmd(IntakeSubsystem intakeSubsystem, Supplier<Boolean> leftexecuteButton,
            Supplier<Boolean> rightexecuteButton) {
        m_intakeSubsystem = intakeSubsystem;
        m_leftexecuteButton = leftexecuteButton;
        m_rightexecuteButton = rightexecuteButton;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_leftexecuteButton.get()) {
            System.out.println(m_leftexecuteButton.get());
            m_intakeSubsystem.setIntakeSpeed(-IntakeConstants.IntakeSpeed);
        } else if (m_rightexecuteButton.get()) {
            System.out.println(m_rightexecuteButton.get());
            m_intakeSubsystem.setIntakeSpeed(IntakeConstants.IntakeSpeed);
        } else {
            m_intakeSubsystem.setIntakeSpeed(0);
        }

    }

}
