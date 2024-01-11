package frc.robot.subsystems.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolenoidSubsystem extends SubsystemBase{
    private static final Compressor comp = new Compressor(null);
     private static final DoubleSolenoid Solenoid = new DoubleSolenoid(null, 0, 1);

    // Turns on the compressor as the robot turns on
    public void teleopInit (){
        comp.enableDigital();
    }
    public static void SetForward(){
        Solenoid.set(DoubleSolenoid.Value.kForward);
    }
    public static void Retract(){
        Solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public static void CompressorOn(){
        comp.enableDigital();
    }
    public static void CompressorOff(){
        comp.disable();
    }
}

