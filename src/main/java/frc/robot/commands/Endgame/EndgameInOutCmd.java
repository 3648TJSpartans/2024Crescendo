package frc.robot.commands.Endgame;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.Trap.TrapSubsystem;

public class EndgameInOutCmd extends Command {
    private final TrapSubsystem m_trapSubsystem;
    private final double m_positionInOut;


    public EndgameInOutCmd(TrapSubsystem trapSubsystem, double positionInOut){
        m_trapSubsystem = trapSubsystem;
        m_positionInOut = positionInOut;
        addRequirements(m_trapSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_trapSubsystem.moveInOut(TrapConstants.kspeed);
    }

    public boolean isFinished(){
        if (TrapSubsystem.getInOutAngle() >= TrapConstants.kpositionInOut){
            return true;
        } 
            return false;
    }
}
