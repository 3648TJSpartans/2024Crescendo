package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Solenoid.SolenoidSubsystem;

public class SolenoidCmd extends Command {
    private final Supplier<Boolean> m_executeButton;
    private final SolenoidSubsystem m_solenoidSubsystem;

    public SolenoidCmd(SolenoidSubsystem m_solenoidSubsystem, Supplier<Boolean> m_executeButton) {
        this.m_solenoidSubsystem = m_solenoidSubsystem;
        this.m_executeButton = m_executeButton;
        addRequirements(m_solenoidSubsystem);
    }

    private final XboxController xbox = new XboxController(0);

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Brings the Pneumatic out
        if (m_executeButton.get()) {
            // Prints the boolean expression of m_executeButton
            System.out.println(m_executeButton.get());
            m_solenoidSubsystem.SetForward();
            // Brings the Pneumatic in
        } else {
            System.out.println(m_executeButton.get());
            m_solenoidSubsystem.Retract();
        }

        // Turn on the compressor(May not be used)
        if (xbox.getYButton()) {
            m_solenoidSubsystem.CompressorOn();
            // Turn off the compressor
        } else if (xbox.getXButton()) {
            m_solenoidSubsystem.CompressorOff();
        }
    }
}
