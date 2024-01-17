package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final Supplier<Boolean> m_ShootButton;
    private final Supplier<Boolean> m_RevButton;
    private final ShooterSubsystem ShooterSubsystem;

    public ShootCommand(ShooterSubsystem m_ShooterSubsystem,Supplier<Boolean> m_ShootButton, Supplier<Boolean> m_RevButton) {
        ShooterSubsystem = m_ShooterSubsystem;
        this.m_ShootButton = m_ShootButton;
        this.m_RevButton = m_RevButton;
        addRequirements(m_ShooterSubsystem);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_RevButton.get()){
            ShooterSubsystem.revShooter();
        }
        if (m_ShootButton.get()){
            ShooterSubsystem.moveShooterIntake();
        }
    }

}
