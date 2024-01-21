package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.TrapSubsystem;

public class EndgameUpDownCmd extends Command {
    private final TrapSubsystem trapSubsystem;
    private final double m_positionUpDown;


    public EndgameUpDownCmd(TrapSubsystem trapSubsystem, double m_positionUpDown){
        this.trapSubsystem = trapSubsystem;
        this.m_positionUpDown = m_positionUpDown;
        addRequirements(trapSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        trapSubsystem.moveUpDown(TrapConstants.speed);
    }

    public boolean isFinished(){
        if (TrapSubsystem.getUpDownPosition() >= TrapConstants.positionUpDown) {
          trapSubsystem.moveUpDown(0);
          return true;  
        }
          return false;
    }
}
