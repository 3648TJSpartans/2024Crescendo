package frc.robot.commands.Endgame;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EndgameConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TrapSubsystem;

public class EndgameCmdGroup extends SequentialCommandGroup {
    private final TrapSubsystem m_trapSubsystem;
    private final ClimberSubsystem m_climberSubsystem;

    public EndgameCmdGroup(TrapSubsystem trapSubsystem, ClimberSubsystem climberSubsystem) {
        m_trapSubsystem = trapSubsystem;
        m_climberSubsystem = climberSubsystem;
        addCommands(
                new EndgameClimberCmd(m_climberSubsystem, ClimberConstants.kClimberDown)
                        .withTimeout(EndgameConstants.kclimberTime),
                new WaitCommand(EndgameConstants.waitTime),
                new EndgameUpDownCmd(m_trapSubsystem, TrapConstants.kpositionUpDown)
                        .withTimeout(EndgameConstants.kTrapUpTime),
                new WaitCommand(EndgameConstants.waitTime),
                new EndgameInOutCmd(m_trapSubsystem, TrapConstants.kpositionInOut)
                        .withTimeout(EndgameConstants.kTrapOutTime),
                new WaitCommand(EndgameConstants.waitTime),
                new EndgameTrackCmd(m_trapSubsystem, TrapConstants.kspeed)
                        .withTimeout(EndgameConstants.kTrapReleaseTime));
    }
}
