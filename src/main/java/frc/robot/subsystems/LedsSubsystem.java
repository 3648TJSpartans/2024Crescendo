package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

import org.ejml.interfaces.decomposition.LUSparseDecomposition;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;

public class LedsSubsystem extends SubsystemBase {

    private final AddressableLEDBuffer m_ledBuffer;

    private final AddressableLED m_led;

    private Timer time = new Timer(); //Not sure if there is a better timer to use
    int count = 0; //Also maybe not wise

    public LedsSubsystem() {
        m_ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
        m_led = new AddressableLED(LedConstants.ledPWMID);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void setColor(int r, int g, int b, int startValue, int endValue) {

        for (int i = startValue; i < endValue; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void intakeColor(DigitalInput IRsenor) {
        if (!IRsenor.get()) {
            setColor(LedConstants.YesNoteRed, LedConstants.YesNoteGreen, LedConstants.YesNoteBlue,
                    LedConstants.topBarLedStart, LedConstants.topBarLedStop);
        } else {

            setColor(LedConstants.NoNoteRed, LedConstants.NoNoteGreen, LedConstants.NoNoteBlue,
                    LedConstants.topBarLedStart, LedConstants.topBarLedStop);

        }

    }

    //Should be called periodically, ramp could be lower when shooting?
    //Some numbers should be changed with constants
    public void pewPew(double ramp) {
        if (time.get() > ramp) {
          setColor(0, 127, 124, 0, 30);
          if (count < 22) {setColor(0, 124, 129, count, count+8);}
          else {
            setColor(0, 124, 129, count, 30);
            setColor(0, 124, 129, 0, (count+8)%30);
          }
          if (count < 23) {setColor(0, 118, 131, count+1, count+7);}
          else if (count < 29) {
            setColor(0, 118, 131, count+1, 30);
            setColor(0, 118, 131, 0, (count+7)%30);
          }
          if (count < 24) {setColor(0, 104, 144, count+2, count+6);}
          else if (count < 28) {
            setColor(0, 104, 144, count+2, 30);
            setColor(0, 104, 144, 0, (count+6)%30);
          }
          if (count < 25) {setColor(0, 92, 152, count+3, count+5);}
          else if (count < 27) {
            setColor(0, 92, 152, count+3, 30);
            setColor(0, 92, 152, 0, (count+5)%30);
          }
          if (count < 26) {setColor(0, 87, 160, count+4, count+4);}
          else {
            setColor(0, 87, 160, 0, (count+4)%30);
          }
          count++;
          if (count > 30) {
            count = 0;
          }
          time.reset();
          time.start();
        }
      }
}
