package frc.robot.subsystems.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolenoidSubsystem extends SubsystemBase{
    private static final Compressor comp = new Compressor(PneumaticsModuleType.REVPH);
     private static final DoubleSolenoid Solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    // Turns on the compressor as the robot turns on
    public void teleopInit (){
        comp.enableDigital();
    }
    public  void SetForward(){
        Solenoid.set(DoubleSolenoid.Value.kForward);
    }
    public  void Retract(){
        Solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public  void CompressorOn(){
        comp.enableDigital();
    }
    public  void CompressorOff(){
        comp.disable();
    }
}