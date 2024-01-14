package frc.robot.subsystems.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolenoidSubsystem extends SubsystemBase{
    // sets the compressor to variable "comp"
    private static final Compressor comp = new Compressor(PneumaticsModuleType.REVPH);
    // set the DoubleSolenoid to variable "Solenoid"
    private static final DoubleSolenoid Solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    
    // Method to turn the compressor on as the robot turns on and is in teleop
    public void teleopInit (){
        comp.enableDigital();
    }
    // Method to bring the solenoid out
    public  void SetForward(){
        Solenoid.set(DoubleSolenoid.Value.kForward);
    }
    // Method to bring the solenoid in
    public  void Retract(){
        Solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    // Method to manually turn on the compressor 
    public  void CompressorOn(){
        comp.enableDigital();
    }
    // Method to manually turn off the compressor
    public  void CompressorOff(){
        comp.disable();
    }
}