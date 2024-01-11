package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Solenoid.SolenoidSubsystem;

public class SolenoidCmd extends Command{
    private final Supplier<Boolean> buttonA, buttonB, buttonY, buttonX;
    private final SolenoidSubsystem solenoidSubsystem;

    public SolenoidCmd(SolenoidSubsystem solenoidSubsystem, Supplier<Boolean> buttonA, Supplier<Boolean> buttonB,
    Supplier<Boolean> buttonY, Supplier<Boolean> buttonX){
        this.solenoidSubsystem = solenoidSubsystem;
        this.buttonA = buttonA;
        this.buttonB = buttonB;
        this.buttonY = buttonY;
        this.buttonX = buttonX;
        //addRequirements(solenoidSubsystem);
    }
    
    private final XboxController xbox = new XboxController(0);  
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Brings the Pneumatic out
            if (xbox.getAButtonPressed()){
            SolenoidSubsystem.SetForward();
        // Brings the Pneumatic in
        } else if(xbox.getBButtonPressed()){
            SolenoidSubsystem.Retract();
        }

        // Turns on the compressor
        if(xbox.getYButton()){
            SolenoidSubsystem.CompressorOn();
        // Turns off the compressor
        } else if (xbox.getXButton()) {
            SolenoidSubsystem.CompressorOff();
        }
    }
}
