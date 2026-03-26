package frc.robot.Subsystems.LEDs;

public class LEDsConstants {
    public static final String SUBSYSTEM_NAME = "LEDs";

    //Rio can only handle one addressable LED, so we have one super long strip, then split it into sections of LEDs
    //TODO: Get actual values for index, and different strip lengths
    public static final int LED_INDEX = 9;
    public static final int MANAGER_LEDS_LENGTH = 60;
    public static final int DRIVE_LEDS_LENGTH = 60;
}
