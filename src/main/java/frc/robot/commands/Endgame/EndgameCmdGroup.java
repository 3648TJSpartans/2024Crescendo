package frc.robot.commands.Endgame;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.Trap.TrapSubsystem;


public class EndgameCmdGroup extends SequentialCommandGroup {
    private final TrapSubsystem m_TrapSubsystem;

    public EndgameCmdGroup(TrapSubsystem m_trapSubsystem) {
        m_TrapSubsystem = m_trapSubsystem;
        addCommands(new EndgameUpDownCmd(m_trapSubsystem, TrapConstants.kpositionUpDown),
                    new WaitCommand(2.5),
                    new EndgameInOutCmd(m_trapSubsystem, TrapConstants.kpositionInOut),
                    new WaitCommand(2.5), 
                    new EndgameTrackCmd(m_trapSubsystem, TrapConstants.kspeedTrack));
    }
}
