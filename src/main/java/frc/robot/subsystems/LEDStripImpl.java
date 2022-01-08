package frc.robot.subsystems;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.interfaces.LEDStrip;
import frc.robot.lib.LEDColour;
import frc.robot.lib.MathUtil;

// LED Strip Subsystem 2020

public class LEDStripImpl implements LEDStrip {
    public AddressableLED ledStrip;
    public AddressableLEDBuffer ledStripBuffer;
    private final int numberOfLEDs;

    public LEDStripImpl(int PWM_Port, int numberOfLEDs) {
        this.numberOfLEDs = numberOfLEDs;

        ledStrip = new AddressableLED(PWM_Port);
        ledStripBuffer = new AddressableLEDBuffer(numberOfLEDs);
        ledStrip.setLength(ledStripBuffer.getLength());

        // Set the data
        ledStrip.setData(ledStripBuffer);
        ledStrip.start();

    }

    @Override
    public void setColour(LEDColour c) {
        for (int i = 0; i < numberOfLEDs; i++) {
            ledStripBuffer.setRGB(i, c.r, c.g, c.b);
        }
        setData();
    }

    @Override
    public void setAlternatingColour(LEDColour c1, LEDColour c2) {
        int alternateNum = numberOfLEDs / 10; // alternate colours every 10% of the strip
        for (int i = 0; i < numberOfLEDs; i++) {
            if ((i / alternateNum) % 2 == 0) {
                setLEDColour(i, c1);
            } else {
                setLEDColour(i, c2);
            }
        }
        setData();
    }

    @Override
    public void setProgressColour(LEDColour c1, LEDColour c2, double percent) {
        percent = MathUtil.clamp(percent, 0, 1);
        int leds = (int) (percent * numberOfLEDs);
        for (int i = 0; i < leds; i++) {
            setLEDColour(i, c1);
        }
        for (int i = leds; i < numberOfLEDs; i++) {
            setLEDColour(i, c2);
        }
        setData();
    }

    @Override
    public void setIdle() {
        for (int i = 0; i < numberOfLEDs; i++) {
            setLEDColour(i, i % 2 == 0 ? LEDColour.YELLOW : LEDColour.GREEN);
        }
        setData();
    }

    private void setData() {
        ledStrip.setData(ledStripBuffer);
    }

    private void setLEDColour(int index, LEDColour c) {
        ledStripBuffer.setRGB(index, c.r, c.g, c.b);
    }
}
