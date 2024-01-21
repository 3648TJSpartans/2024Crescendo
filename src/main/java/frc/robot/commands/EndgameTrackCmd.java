package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.TrapSubsystem;

public class EndgameTrackCmd extends Command {
    private final TrapSubsystem trapSubsystem;
    private final double m_speedTrack;
    private long m_startTime;


    public EndgameTrackCmd(TrapSubsystem trapSubsystem, double m_speedTrack){
        this.trapSubsystem = trapSubsystem;
        this.m_speedTrack = m_speedTrack;
        addRequirements(trapSubsystem);
    }

    @Override
    public void initialize(){
        m_startTime = System.currentTimeMillis();    
    }

    @Override
    public void execute(){
        double speedTrack = m_speedTrack;
        trapSubsystem.moveTrack(TrapConstants.speed);
    }

    public boolean isFinished(){
        if(System.currentTimeMillis() > m_startTime + 3000){
            trapSubsystem.moveTrack(0);
            return true;
        }
        return false;
    }
}
