package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;

public final class ShooterConstants {

	public static final String SUBSYSTEM_NAME = "Shooter";
	// Preset positions
	public static final Angle HOOD_MIN_ANGLE = Degrees.of(0); // TODO: get real value
	public static final Angle HOOD_MAX_ANGLE = Degrees.of(180); // TODO: get real value
	
	public static final Angle FIXED_SHOT_ANGLE = Degrees.of(45); //TODO: get good value
	public static final AngularVelocity FIXED_SHOT_SPEED = RotationsPerSecond.of(78); //TODO: get good value
	public static final Angle REVERSE_SHOT_ANGLE = Degrees.of(10); //TODO: get good value
	public static final AngularVelocity REVERSE_SHOT_SPEED = RotationsPerSecond.of(-5);

	public static final Angle STANDBY_ANGLE = Degrees.of(45); //TODO: get good value
	public static final AngularVelocity STANDBY_SPEED = RotationsPerSecond.of(500); //TODO: get good value

	/** Maximum iterations for convergence */
	public static final int SOLVER_ITERATIONS = 10;
	
	/** Minimum distance epsilon for comparisons (meters) */
	public static final double DISTANCE_EPSILON = 0.001;
	public static final double CONVERGENCE_THRESHOLD = 0.001; // meters
	
	// ========== FLYWHEEL SPEEDS ==========
	// Each table uses a fixed flywheel speed
	
	public static final AngularVelocity NEAR_FLYWHEEL_SPEED = RotationsPerSecond.of(40.0);
	public static final AngularVelocity MEDIUM_FLYWHEEL_SPEED = RotationsPerSecond.of(60.0);
	public static final AngularVelocity FAR_FLYWHEEL_SPEED = RotationsPerSecond.of(80.0);
	
	// ========== TARGET POSES ==========
	
	public static final Pose2d HUB_POSE = new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0));
	public static final Pose2d SHALLOW_ALLIANCE_POSE = new Pose2d(1.8, 8.2, Rotation2d.fromDegrees(0));
	public static final Pose2d DEEP_ALLIANCE_POSE = new Pose2d(5.0, 8.0, Rotation2d.fromDegrees(0));

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
			case SIM -> new PIDController(0.0032, 0.0, 0.0);
			case TESTING -> new PIDController(0.1, 0, 0);
		}; //TODO: tune
	public static final Supplier<SimpleMotorFeedforward> WHEEL_FEEDFORWARD = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new SimpleMotorFeedforward(0.1, 1, 0.001);
			case SIM -> new SimpleMotorFeedforward(0.0, 0.001583, 0.0);
			case TESTING -> new SimpleMotorFeedforward(0.1, 1, 0.001);
		}; //TODO: tune

	// Placeholder positions; replace with real field measurements, Define based on alliance side
	public static final Transform3d ROBOT_TO_SHOOTER = new Transform3d(-8.937500, -4.4083305, 19, new Rotation3d(0, 0, Math.PI / 2));
	public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.625, 4.08, Rotation2d.kZero);
	public static final Pose2d RED_HUB_POSE = new Pose2d(11.92, 4.08, Rotation2d.kZero);

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
