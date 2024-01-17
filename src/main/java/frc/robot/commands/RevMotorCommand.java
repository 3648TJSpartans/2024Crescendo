package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevMotorCommand extends Command {
    private final Supplier<Boolean> m_ShootButton;
    private final ShooterSubsystem ShooterSubsystem;

    public RevMotorCommand(ShooterSubsystem m_ShooterSubsystem, Supplier<Boolean> m_ShootButton) {
        ShooterSubsystem = m_ShooterSubsystem;
        this.m_ShootButton = m_ShootButton;
        addRequirements(m_ShooterSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_ShootButton.get()) {
            ShooterSubsystem.revShooter();

        }

    }

}
