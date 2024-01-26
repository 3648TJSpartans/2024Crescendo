package frc.robot.commands.Endgame;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.TrapSubsystem;

public class EndgameUpDownCmd extends Command {
    private final TrapSubsystem m_trapSubsystem;
    private final double m_positionUpDown;

    public EndgameUpDownCmd(TrapSubsystem trapSubsystem, double positionUpDown) {
        m_trapSubsystem = trapSubsystem;
        m_positionUpDown = positionUpDown;
        addRequirements(m_trapSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_trapSubsystem.moveUpDown(TrapConstants.kspeed);
    }

    public boolean isFinished() {
        if (TrapSubsystem.getUpDownPosition() >= TrapConstants.kpositionUpDown) {
            return true;
        }
        return false;
    }
}
