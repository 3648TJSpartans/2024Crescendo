package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberJoystickCmd extends Command {
    private final ClimberSubsystem m_climberSubsystem;
    private final Supplier<Double> m_speed;

    public ClimberJoystickCmd(ClimberSubsystem climberSubsystem, Supplier<Double> speed){
        m_climberSubsystem = climberSubsystem;
        m_speed = speed;
        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        double speed = m_speed.get();
        m_climberSubsystem.MoveClimber(speed);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
