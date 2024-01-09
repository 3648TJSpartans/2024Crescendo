import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;

public class Solenoid {
    private final Compressor comp = new Compressor(null);
    private final DoubleSolenoid Solenoid = new DoubleSolenoid(null, 0, 1);

    private final XboxController xbox = new XboxController(0);

    public void teleopInit (){
        comp.enableDigital();
    }

    public void teleopPeriodic(){
        if (xbox.getAButtonPressed()){
            Solenoid.set(DoubleSolenoid.Value.kForward);
        } else if(xbox.getBButtonPressed()){
            Solenoid.set(DoubleSolenoid.Value.kReverse);
        }

        if(xbox.getYButton()){
            comp.enableDigital();
        } else if (xbox.getXButton()) {
            comp.disable();
        }
     }
}
