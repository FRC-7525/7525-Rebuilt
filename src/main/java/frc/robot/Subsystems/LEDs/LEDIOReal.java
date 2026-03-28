package frc.robot.Subsystems.LEDs;

import static frc.robot.Subsystems.LEDs.LEDsConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;

public class LEDIOReal implements LEDIO {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDBufferView underglowBuffer;
    private AddressableLEDBufferView leftSideBuffer;
    private AddressableLEDBufferView spindexerBuffer;
    private AddressableLEDBufferView rightSideBuffer;

    public LEDIOReal() {
        ledStrip = new AddressableLED(LED_INDEX);
        ledBuffer = new AddressableLEDBuffer(UNDERGLOW_LEDS_LENGTH + LEFT_SIDE_LEDS_LENGTH + SPINDEXER_LEDS_LENGTH + RIGHT_SIDE_LEDS_LENGTH);

        ledStrip.setLength(UNDERGLOW_LEDS_LENGTH + LEFT_SIDE_LEDS_LENGTH + SPINDEXER_LEDS_LENGTH + RIGHT_SIDE_LEDS_LENGTH);

        underglowBuffer = ledBuffer.createView(0, UNDERGLOW_LEDS_LENGTH - 1);
        leftSideBuffer = ledBuffer.createView(UNDERGLOW_LEDS_LENGTH, UNDERGLOW_LEDS_LENGTH + LEFT_SIDE_LEDS_LENGTH - 1);
        spindexerBuffer = ledBuffer.createView(UNDERGLOW_LEDS_LENGTH + LEFT_SIDE_LEDS_LENGTH, UNDERGLOW_LEDS_LENGTH + LEFT_SIDE_LEDS_LENGTH + SPINDEXER_LEDS_LENGTH - 1);
        rightSideBuffer = ledBuffer.createView(UNDERGLOW_LEDS_LENGTH + LEFT_SIDE_LEDS_LENGTH + SPINDEXER_LEDS_LENGTH, UNDERGLOW_LEDS_LENGTH + LEFT_SIDE_LEDS_LENGTH + SPINDEXER_LEDS_LENGTH + RIGHT_SIDE_LEDS_LENGTH);

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public void setManagerPattern(LEDPattern pattern) {
        pattern.applyTo(leftSideBuffer);
        pattern.applyTo(spindexerBuffer);
        pattern.applyTo(rightSideBuffer);
    }

    @Override
    public void setDrivePattern(LEDPattern pattern) {
        pattern.applyTo(underglowBuffer);
    }

    @Override
    public void setLEDData() {
        ledStrip.setData(ledBuffer);
    }
}
