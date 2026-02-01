package frc.robot.Subsystems.Shooter;

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
	public static final AngularVelocity FIXED_SHOT_SPEED = RotationsPerSecond.of(150); //TODO: get good value

	public static final Angle STANDBY_ANGLE = Degrees.of(45); //TODO: get good value
	public static final AngularVelocity STANDBY_SPEED = RotationsPerSecond.of(500); //TODO: get good value

	// Numerical constants (moved from magic literals)
	public static final double SOLVER_EPSILON = 1e-6;
	public static final int SOLVER_ITERATIONS = 4;

	// Tolerances
	public static final double WHEEL_VELOCITY_TOLERANCE = 50.0; // in RotationsPerSecond units
	public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 0.01; // degrees

	// Simulation / physical defaults
	public static final double FLYWHEEL_MOI = 0.00117056; // Roughly accurate for flywheel
	public static final double FLYWHEEL_GEARING = 1.0; // Also roughly accurate
	public static final double HOOD_MOI = 0.0001;
	public static final double HOOD_GEARING = 1.0;
	public static final double HOOD_ARM_LENGTH_METERS = 0.2;

	// State defaults
	public static final AngularVelocity REVERSE_WHEEL_SPEED = RotationsPerSecond.of(-100);

	public static final int LEFT_SHOOTER_MOTOR_ID = 37;
	public static final int RIGHT_SHOOTER_MOTOR_ID = 38;
	public static final int HOOD_MOTOR_ID = 39;

	public static final Supplier<PIDController> HOOD_PID = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1, 0, 0);
			case SIM -> new PIDController(0.04, 0, 0.001); // Tuned in sim
			case TESTING -> new PIDController(1, 0, 0);
		}; //TODO: tune
	public static final Supplier<PIDController> WHEEL_PID = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(0.1, 0, 0);
			case SIM -> new PIDController(0.0077, 0, 0.00013);
			case TESTING -> new PIDController(0.1, 0, 0);
		}; //TODO: tune
	public static final Supplier<SimpleMotorFeedforward> WHEEL_FEEDFORWARD = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new SimpleMotorFeedforward(0.1, 0.01, 0.001);
			case SIM -> new SimpleMotorFeedforward(0.11, 0.1, 0.0);
			case TESTING -> new SimpleMotorFeedforward(0.1, 0.01, 0.001);
		}; //TODO: tune

	// Placeholder positions; replace with real field measurements, Define based on alliance side
	public static final Pose2d HUB_POSITION = new Pose2d(0.0, 0.0, new edu.wpi.first.math.geometry.Rotation2d());

	public static final Pose2d ALLIANCE_BASKET_POSITION = new Pose2d(0.0, 0.0, new edu.wpi.first.math.geometry.Rotation2d());

	// Shot sample data for lookup tables (placeholder/example values)
	public static record ShotSampleData(double distanceMeters, Angle hoodAngle, AngularVelocity flywheelSpeed, double timeOfFlightSeconds) {}

	public static final java.util.List<ShotSampleData> HUB_SHOT_SAMPLES = java.util.List.of(
		new ShotSampleData(2.0, Degrees.of(30), RotationsPerSecond.of(20), 0.50),
		new ShotSampleData(3.0, Degrees.of(35), RotationsPerSecond.of(25), 0.55),
		new ShotSampleData(4.0, Degrees.of(40), RotationsPerSecond.of(30), 0.60),
		new ShotSampleData(5.0, Degrees.of(45), RotationsPerSecond.of(35), 0.65)
	);

	public static final java.util.List<ShotSampleData> ALLIANCE_SHOT_SAMPLES = java.util.List.of(
		new ShotSampleData(2.0, Degrees.of(28), RotationsPerSecond.of(19), 0.52),
		new ShotSampleData(3.0, Degrees.of(33), RotationsPerSecond.of(24), 0.56),
		new ShotSampleData(4.0, Degrees.of(38), RotationsPerSecond.of(29), 0.61),
		new ShotSampleData(5.0, Degrees.of(44), RotationsPerSecond.of(34), 0.66)
	);
}
