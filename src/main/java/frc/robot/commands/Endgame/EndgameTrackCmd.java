package frc.robot.commands.Endgame;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.Trap.TrapSubsystem;

public class EndgameTrackCmd extends Command {
    private final TrapSubsystem m_trapSubsystem;
    private final double m_speedTrack;
    private long m_startTime;


    public EndgameTrackCmd(TrapSubsystem trapSubsystem, double speedTrack){
        m_trapSubsystem = trapSubsystem;
        m_speedTrack = speedTrack;
        addRequirements(m_trapSubsystem);
    }

    @Override
    public void initialize(){
        m_startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        m_trapSubsystem.moveTrack(TrapConstants.kspeed);
    }

    public boolean isFinished(){
        if(System.currentTimeMillis() > m_startTime + 3000){
            return true;
        }
        return false;
    }
}
