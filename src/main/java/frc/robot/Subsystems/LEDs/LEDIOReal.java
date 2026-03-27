package frc.robot.Subsystems.LEDs;

import static frc.robot.Subsystems.LEDs.LEDsConstants.DRIVE_LEDS_LENGTH;
import static frc.robot.Subsystems.LEDs.LEDsConstants.LED_INDEX;
import static frc.robot.Subsystems.LEDs.LEDsConstants.MANAGER_LEDS_LENGTH;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;

public class LEDIOReal implements LEDIO {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDBufferView managerLEDBuffer;
    private AddressableLEDBufferView driveLEDBuffer;

    public LEDIOReal() {
        ledStrip = new AddressableLED(LED_INDEX);
        ledBuffer = new AddressableLEDBuffer(DRIVE_LEDS_LENGTH + MANAGER_LEDS_LENGTH);

        managerLEDBuffer = ledBuffer.createView(0, MANAGER_LEDS_LENGTH - 1);
        driveLEDBuffer = ledBuffer.createView(MANAGER_LEDS_LENGTH, DRIVE_LEDS_LENGTH - 1);

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public void setManagerPattern(LEDPattern pattern) {
        pattern.applyTo(managerLEDBuffer);
    }

    @Override
    public void setDrivePattern(LEDPattern pattern) {
        pattern.applyTo(driveLEDBuffer);
    }

    @Override
    public void setLEDData() {
        ledStrip.setData(ledBuffer);
    }
}
