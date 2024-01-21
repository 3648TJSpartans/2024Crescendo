package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.TrapSubsystem;

public class EndgameInOutCmd extends Command {
    private final TrapSubsystem trapSubsystem;
    private final double m_positionInOut;


    public EndgameInOutCmd(TrapSubsystem trapSubsystem, double m_positionInOut){
        this.trapSubsystem = trapSubsystem;
        this.m_positionInOut = m_positionInOut;
        addRequirements(trapSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        trapSubsystem.moveInOut(TrapConstants.speed);
    }

    public boolean isFinished(){
        if (TrapSubsystem.getInOutAngle() >= TrapConstants.positionInOut){
            trapSubsystem.moveInOut(0);
            return true;
        } 
            return false;
    }
}
