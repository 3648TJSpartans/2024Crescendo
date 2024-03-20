package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

import org.ejml.interfaces.decomposition.LUSparseDecomposition;

import edu.wpi.first.wpilibj.AddressableLED;

public class LedsSubsystem extends SubsystemBase {

    private final AddressableLEDBuffer m_ledBuffer;

    private final AddressableLED m_led;

    public LedsSubsystem() {
        m_ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
        m_led = new AddressableLED(LedConstants.ledPWMID);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void setColorRGB(int r, int g, int b, int startValue, int endValue) {
        for (int i = startValue; i < endValue; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setColorHSV(int h, int s, int v, int startValue, int endValue) {
        for (int i = startValue; i < endValue; i++) {
            m_ledBuffer.setHSV(i, h, s, v);
        }
        m_led.setData(m_ledBuffer);
    }

    public void pewPewWave(double shift, int delta, int startValue, int endValue, boolean inverted) {
        // For best results, set shift to follow 0 < val < 1
        for (int i = startValue; i < endValue; i++) {
            m_ledBuffer.setRGB(i, 255, (int) Math.round(delta * Math.cos(Math.PI
                    * (((double) i - startValue) / ((double) endValue - startValue) + ((inverted ? -1 : 1) * shift)))
                    * Math.cos(
                            Math.PI * (((double) i - startValue) / ((double) endValue - startValue)
                                    + ((inverted ? -1 : 1) * shift)))),
                    0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void intakeWave(double shift, double gradient, int delta, int startValue, int endValue, boolean inverted) {
        // To invert color scheme, change gradient to opposite sign
        for (int i = startValue; i < endValue; i++) {
            m_ledBuffer.setHSV(i,
                    (int) Math.round(
                            delta / 2 * (Math.tanh(
                                    (((double) i - startValue) / ((double) endValue - startValue)
                                            - (inverted ? -1 : 1) * shift + (inverted ? -11 : 3)) / gradient)
                                    + 1))
                            + 60,
                    1, 1);
        }
    }
}
