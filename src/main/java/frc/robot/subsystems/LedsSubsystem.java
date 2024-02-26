package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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

    public void setColor(int rgb[], int startValue, int endValue) {

        for (int i = startValue; i < endValue; i++) {
            m_ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setIntakeColor(Command intakeCmd) {
        if (intakeCmd.isFinished()) {
            setColor(LedConstants.yesNoteRGB, LedConstants.topBarLedStart, LedConstants.topBarLedStop);
        } else if (intakeCmd.isScheduled()) {
            setColor(LedConstants.intakeRunningRGB, LedConstants.topBarLedStart, LedConstants.topBarLedStop);
        } else {
            setColor(LedConstants.noNoteRGB, LedConstants.topBarLedStart, LedConstants.topBarLedStop);
        }
    }
}
