package frc.robot.Subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.Shooter;

public class LEDsConstants {

	public static final String SUBSYSTEM_NAME = "LEDs";

	//Rio can only handle one addressable LED, so we have one super long strip, then split it into sections of LEDs
	//TODO: Get actual values for index, and different strip lengths
	public static final int LED_INDEX = 9;
	public static final int LEFT_SIDE_LEDS_LENGTH = 24;
	public static final int RIGHT_SIDE_LEDS_LENGTH = 26;
	public static final int UNDERGLOW_LEDS_LENGTH = 30;

	public static final HSV PIONEERS_ORANGE = new HSV(13, 204, 231);
	public static final HSV PIONEERS_BLUE = new HSV(107, 140, 200);

	//Disabled constants
	public static final Time DISABLED_BREATH_PERIOD = Seconds.of(2);
	public static final LEDPattern DISABLED_PATTERN = LEDPattern.solid(Color.kRed).breathe(DISABLED_BREATH_PERIOD);

	//Manager State LEDs Constants
	//Idle constants
	public static final double IDLE_SCROLL_SPEED = 25;
	public static final LEDPattern IDLE_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.fromHSV(PIONEERS_BLUE.hue, PIONEERS_BLUE.sat, PIONEERS_BLUE.val), Color.fromHSV(PIONEERS_ORANGE.hue, PIONEERS_ORANGE.sat, PIONEERS_ORANGE.val)).scrollAtRelativeSpeed(
		Percent.per(Seconds).of(IDLE_SCROLL_SPEED)
	);

	//Intaking constants
	public static final double INTAKING_SCROLL_SPEED = 25;
	public static final LEDPattern INTAKING_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, Color.kBlack, Color.kRoyalBlue).scrollAtRelativeSpeed(Percent.per(Seconds).of(INTAKING_SCROLL_SPEED));

	//Wind-up constants
	public static final double SETPOINT_DEVIATION = 1;
	public static final Time WOUND_UP_BREATH_PERIOD = Seconds.of(1);
	public static final LEDPattern WINDING_UP_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kYellow).mask(
		LEDPattern.progressMaskLayer(() -> Shooter.getInstance().getWheelVelocity().in(RotationsPerSecond) / Shooter.getInstance().getWheelSetpoint().in(RotationsPerSecond))
	);
	public static final LEDPattern WOUND_UP_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kYellow).breathe(WOUND_UP_BREATH_PERIOD);

	//Shooting constants
	public static final double SHOOTING_SCROLL_SPEED = 50;
	public static final LEDPattern SHOOTING_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, Color.kBlack, Color.kYellow).scrollAtRelativeSpeed(Percent.per(Seconds).of(SHOOTING_SCROLL_SPEED));

	//Shuttling constants
	public static final double SHUTTLING_SCROLL_SPEED = 50;
	public static final LEDPattern SHUTTLING_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, Color.kRoyalBlue, Color.kRed).scrollAtRelativeSpeed(Percent.per(Seconds).of(SHUTTLING_SCROLL_SPEED));

	//Drive state LEDs constants
	//Snake Drive constants
	public static final double SNAKE_SCROLL_SPEED = 25;
	public static final LEDPattern SNAKE_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, Color.kBlack, Color.kMediumSeaGreen).scrollAtRelativeSpeed(Percent.per(Seconds).of(SNAKE_SCROLL_SPEED));

	//Normal Drive constants
	//TODO: Come up with a cooler pattern
	public static final Time NORMAL_BREATH_PERIOD = Seconds.of(1);
	public static final LEDPattern NORMAL_PATTERN = LEDPattern.solid(Color.kMediumSeaGreen).breathe(NORMAL_BREATH_PERIOD);

	//Autoalign constants
	public static final double AUTOALIGN_SCROLL_SPEED = 25;
	public static final LEDPattern AUTOALIGN_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kOrangeRed, Color.kFirebrick).scrollAtRelativeSpeed(Percent.per(Seconds).of(AUTOALIGN_SCROLL_SPEED));

	//Aimlock constants
	public static final Time AIMLOCK_BREATHE_PERIOD = Seconds.of(1);
	public static final double LOCKED_IN_DEGREES = 5;
	public static final LEDPattern AIMLOCKING_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kForestGreen, Color.kRoyalBlue).mask(LEDPattern.progressMaskLayer(() -> (180 - Drive.getInstance().shooterToTargetAngle) / 180));
	public static final LEDPattern AIMLOCKED_PATTERN = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kForestGreen, Color.kRoyalBlue).breathe(AIMLOCK_BREATHE_PERIOD);

	//X Pose constants
	public static final Time X_POSE_BREATH_PERIOD = Seconds.of(1);
	public static final LEDPattern X_POSE_PATTERN = LEDPattern.solid(Color.kRed).breathe(DISABLED_BREATH_PERIOD);
}
