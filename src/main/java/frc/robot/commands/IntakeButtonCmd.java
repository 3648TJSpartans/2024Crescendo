package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeButtonCmd extends Command {
    private final Supplier<Boolean> m_leftexecuteButton;
    private final Supplier<Boolean> m_rightexecuteButton;
    private final IntakeSubsystem IntakeSubsystem;

    public IntakeButtonCmd(IntakeSubsystem IntakeSubsystem, Supplier<Boolean> m_leftexecuteButton,
            Supplier<Boolean> m_rightexecuteButton) {
        this.IntakeSubsystem = IntakeSubsystem;
        this.m_leftexecuteButton = m_leftexecuteButton;
        this.m_rightexecuteButton = m_rightexecuteButton;
        addRequirements(IntakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_leftexecuteButton.get()) {
            System.out.println(m_leftexecuteButton.get());
            IntakeSubsystem.setIntakeSpeed(-IntakeConstants.IntakeSpeed);
        } else if (m_rightexecuteButton.get()) {
            System.out.println(m_rightexecuteButton.get());
            IntakeSubsystem.setIntakeSpeed(IntakeConstants.IntakeSpeed);
        } else {
            IntakeSubsystem.setIntakeSpeed(0);
        }

    }

}
