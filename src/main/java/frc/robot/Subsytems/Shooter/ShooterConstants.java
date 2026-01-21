package frc.robot.Subsytems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;

public final class ShooterConstants {

	public static final String SUBSYSTEM_NAME = "Shooter";

	public static final Angle FIXED_SHOT_ANGLE = Degrees.of(45); //TODO: get good value
	public static final AngularVelocity FIXED_SHOT_SPEED = RotationsPerSecond.of(1000); //TODO: get good value

	public static final int LEFT_SHOOTER_MOTOR_ID = 10;
	public static final int RIGHT_SHOOTER_MOTOR_ID = 11;
	public static final int HOOD_MOTOR_ID = 12;

	public static final Supplier<PIDController> HOOD_PID = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1, 0, 0);
			case SIM -> new PIDController(0.05, 0, 0);
			case TESTING -> new PIDController(1, 0, 0);
		}; //TODO: tune
	public static final Supplier<PIDController> WHEEL_PID = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(0.1, 0, 0);
			case SIM -> new PIDController(0.01, 0, 0);
			case TESTING -> new PIDController(0.1, 0, 0);
		}; //TODO: tune
	public static final Supplier<SimpleMotorFeedforward> WHEEL_FEEDFORWARD = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new SimpleMotorFeedforward(0.1, 0.01, 0.001);
			case SIM -> new SimpleMotorFeedforward(0.05, 0.005, 0.0005);
			case TESTING -> new SimpleMotorFeedforward(0.1, 0.01, 0.001);
		}; //TODO: tune

	// Placeholder positions; replace with real field measurements, Define based on alliance side
	public static final Pose2d HUB_POSITION = new Pose2d(0.0, 0.0, new edu.wpi.first.math.geometry.Rotation2d());

	public static final Pose2d ALLIANCE_BASKET_POSITION = new Pose2d(0.0, 0.0, new edu.wpi.first.math.geometry.Rotation2d());
}
