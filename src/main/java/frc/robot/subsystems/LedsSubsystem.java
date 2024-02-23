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
}
