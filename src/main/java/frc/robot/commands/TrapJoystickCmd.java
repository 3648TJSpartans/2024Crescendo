package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapSubsystem;

public class TrapJoystickCmd extends Command {
    private final TrapSubsystem m_trapSubsystem;
    private Supplier<Double> m_speedUpDown;
    private Supplier<Double> m_speedInOut;

    public TrapJoystickCmd(TrapSubsystem trapSubsystem, Supplier<Double> speedUpDown, Supplier<Double> speedInOut) {
        m_trapSubsystem = trapSubsystem;
        m_speedUpDown = speedUpDown;
        m_speedInOut = speedInOut;
        addRequirements(m_trapSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speedUpDown = m_speedUpDown.get();
        double speedInOut = m_speedInOut.get();
        m_trapSubsystem.moveUpDown(speedUpDown);
        // m_trapSubsystem.moveInOut(speedInOut);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
