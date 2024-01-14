package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Solenoid.SolenoidSubsystem;

public class SolenoidCmd extends Command {
    // Creates the buttons as supplier boolean values to excute pneumatic and compressor controls
    private final Supplier<Boolean> m_executeButton, m_CompressorOn, m_CompressorOff;
    // Creates "m_solenoidSubsystem" to use to call methods created in SolenoidSubsystem file
    private final SolenoidSubsystem m_solenoidSubsystem;

    /** Creates constructor that creates the instance of "SolenoidCmd()" that is called in RobotContainer file 
        - sets the intial variables equal to parameters in the constructor
        - "addrequirements()" intializes m_solenoidSubsystem in the contructor, being called in the RobotContainer
    */
    public SolenoidCmd(SolenoidSubsystem m_solenoidSubsystem, Supplier<Boolean> m_executeButton, Supplier<Boolean> m_CompressorOn, 
    Supplier<Boolean> m_CompressorOff) {
        this.m_solenoidSubsystem = m_solenoidSubsystem;
        this.m_executeButton = m_executeButton;
        this.m_CompressorOn = m_CompressorOn;
        this.m_CompressorOff = m_CompressorOff;
        addRequirements(m_solenoidSubsystem);
    }


    @Override
    // "intialize()" is called when the program intially runs
    public void initialize() {
        m_solenoidSubsystem.CompressorOn();
    }

    @Override
    // "execute()" is called continuously as the program runs
    public void execute() {
        // ".get()" converts a supplier boolean to a regular boolean
        if (m_executeButton.get()) {
            /** Uses ".setForward()" Method created in SolenoidSubsystem.java to bring the Pneumatic out
            - Prints the boolean expression of m_executeButton(Debug Line)
            */ 
            System.out.println(m_executeButton.get());
            m_solenoidSubsystem.SetForward();
        } else {
            // Uses ".Retract()" Method created in SolenoidSubsystem.java to bring the Pneumatic in
            System.out.println(m_executeButton.get());
            m_solenoidSubsystem.Retract();
        }

        if (m_CompressorOn.get()) {
            // Uses ".CompressorOn()" Method created in SolenoidSubsystem.java to turn on the compressor
            System.out.println(m_CompressorOn.get());
            m_solenoidSubsystem.CompressorOn();
        } else if (m_CompressorOff.get()) {
            // Uses ".CompressorOff()" Method created in SolenoidSubsystem.java to turn off the compressor
            System.out.println(m_CompressorOff.get());
            m_solenoidSubsystem.CompressorOff();
        }
    }
}