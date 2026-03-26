package frc.robot.Subsystems.LEDs;

public class LEDsConstants {
    public static final String SUBSYSTEM_NAME = "LEDs";

    //Rio can only handle one addressable LED, so we have one super long strip, then split it into sections of LEDs
    //TODO: Get actual values for index, and different strip lengths
    public static final int LED_INDEX = 9;
    public static final int MANAGER_LEDS_LENGTH = 60;
    public static final int DRIVE_LEDS_LENGTH = 60;

    public static final HSV PIONEERS_ORANGE = new HSV(13, 204, 231);
    public static final HSV PIONEERS_BLUE = new HSV(107, 140, 200);

    //Disabled constants
    public static final double DISABLED_BREATH_PERIOD = 2;

    //Idle constants
    public static final double IDLE_SCROLL_SPEED = 0.25;

    //Intaking constants
    public static final double INTAKING_SCROLL_SPEED = 0.25;
}
