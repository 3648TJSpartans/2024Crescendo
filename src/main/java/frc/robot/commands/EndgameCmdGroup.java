package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.TrapSubsystem;


public class EndgameCmdGroup extends SequentialCommandGroup {
    private final TrapSubsystem m_TrapSubsystem;

    public EndgameCmdGroup(TrapSubsystem trapSubsystem) {
        m_TrapSubsystem = trapSubsystem;
        addCommands(new EndgameUpDownCmd(trapSubsystem, TrapConstants.positionUpDown),
                    new WaitCommand(5),
                    new EndgameInOutCmd(trapSubsystem, TrapConstants.positionInOut),
                    new WaitCommand(5), 
                    new EndgameTrackCmd(trapSubsystem, TrapConstants.speedTrack));
    }
}
