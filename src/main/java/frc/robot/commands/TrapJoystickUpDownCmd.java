package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trap.TrapSubsystem;

public class TrapJoystickUpDownCmd extends Command {
    private final TrapSubsystem m_trapSubsystem;
    private Supplier<Double> m_speed;


    public TrapJoystickUpDownCmd(TrapSubsystem trapSubsystem, Supplier<Double> speed){
        m_trapSubsystem = trapSubsystem;
        m_speed = speed;
        addRequirements(m_trapSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        double speed = m_speed.get();
        m_trapSubsystem.moveUpDown(speed);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
