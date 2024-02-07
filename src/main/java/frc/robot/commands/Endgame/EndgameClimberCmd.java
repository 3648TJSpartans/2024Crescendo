package frc.robot.commands.Endgame;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class EndgameClimberCmd extends Command {
    private final ClimberSubsystem m_climberSubsystem;
    private final double m_climber_position;

    public EndgameClimberCmd(ClimberSubsystem climberSubsystem, double climber_position) {
        m_climberSubsystem = climberSubsystem;
        m_climber_position = climber_position;
        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_climberSubsystem.setClimberPosition(m_climber_position);
    }

    @Override
    public void end(boolean interrupted) {
    }

    // @Override
    // public boolean isFinished() {
    //     if (ClimberSubsystem.getClimberPosition() >= ClimberConstants.kClimberDown) {
    //         return true;
    //     }
    //     return false;
    // }

}
