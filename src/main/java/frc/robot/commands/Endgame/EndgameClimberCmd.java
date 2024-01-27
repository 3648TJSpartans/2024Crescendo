package frc.robot.commands.Endgame;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class EndgameClimberCmd extends Command {
    private final ClimberSubsystem m_climberSubsystem;
    private final double m_speed;

    public EndgameClimberCmd(ClimberSubsystem climberSubsystem, double speed){
        m_climberSubsystem = climberSubsystem;
        m_speed = speed;
        addRequirements(m_climberSubsystem);
    }

    @ Override
    public void initialize(){}

    @Override
    public void execute(){
        double speed = m_speed;
        m_climberSubsystem.MoveClimber(speed);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        if(ClimberSubsystem.getClimberPosition() >= ClimberConstants.kpositionHeight){
            return true;
        }
        return false;
    }

}
