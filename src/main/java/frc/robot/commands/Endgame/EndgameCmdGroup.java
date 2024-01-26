package frc.robot.commands.Endgame;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.TrapSubsystem;

public class EndgameCmdGroup extends SequentialCommandGroup {
    private final TrapSubsystem m_trapSubsystem;

    public EndgameCmdGroup(TrapSubsystem trapSubsystem) {
        m_trapSubsystem = trapSubsystem;
        addCommands(new EndgameUpDownCmd(m_trapSubsystem, TrapConstants.kpositionUpDown),
                new WaitCommand(TrapConstants.kTrapTime),
                new EndgameInOutCmd(m_trapSubsystem, TrapConstants.kpositionInOut),
                new WaitCommand(TrapConstants.kTrapTime),
                new EndgameTrackCmd(m_trapSubsystem, TrapConstants.kspeed));
    }
}
