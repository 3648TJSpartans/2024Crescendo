package frc.robot.commands.Endgame;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapSubsystem;

public class EndgameTrackDisengagedCmd extends Command {
    private final TrapSubsystem m_trapSubsystem;
    private final double m_trackPos;
    private long m_startTime;

    public EndgameTrackDisengagedCmd(TrapSubsystem trapSubsystem, double trackPos) {
        m_trapSubsystem = trapSubsystem;
        m_trackPos = trackPos;
        addRequirements(m_trapSubsystem);
    }

    @Override
    public void initialize() {
        m_startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        m_trapSubsystem.setTrack(m_trackPos);
    }

    @Override
    public void end(boolean interrupted) {
    }

    // public boolean isFinished() {
    // if (System.currentTimeMillis() > m_startTime + 3000) {
    // return true;
    // }
    // return false;
    // }
}
