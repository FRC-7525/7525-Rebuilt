package frc.robot.Subsystems.LEDs;

import static frc.robot.Subsystems.LEDs.LEDsConstants.*;
import org.team7525.subsystem.Subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDs extends Subsystem<LEDStates> {
    private static LEDs instance;
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDBufferView managerLEDBuffer;
    private AddressableLEDBufferView driveLEDBuffer;

    public static LEDs getInstance() {
        if (instance == null) {
            new LEDs();
        }
        return instance;
    }

    private LEDs() {
        super(SUBSYSTEM_NAME, LEDStates.IDLE);

        ledStrip = new AddressableLED(LED_INDEX);
        ledBuffer = new AddressableLEDBuffer(DRIVE_LEDS_LENGTH + MANAGER_LEDS_LENGTH);

        managerLEDBuffer = ledBuffer.createView(0, MANAGER_LEDS_LENGTH - 1);
        driveLEDBuffer = ledBuffer.createView(MANAGER_LEDS_LENGTH, DRIVE_LEDS_LENGTH - 1);

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }
    
    @Override
    public void runState() {
        //TODO: Might move these pattern applies to stateInit, cuz idk how expensive they are
        getState().getPattern().applyTo(managerLEDBuffer);
        //TODO: I think just make an if statement for drive patterns and dont use subsystem states, but seems like a cooked way to do it
        getState().getPattern().applyTo(driveLEDBuffer); 

        ledStrip.setData(ledBuffer);
    }
}
