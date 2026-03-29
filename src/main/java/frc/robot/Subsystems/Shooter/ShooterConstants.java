package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;

public final class ShooterConstants {

	public static final String SUBSYSTEM_NAME = "Shooter";
	// Preset positions
	public static final Angle HOOD_MIN_ANGLE = Degrees.of(-4.8025772);
	public static final Angle HOOD_MAX_ANGLE = Degrees.of(-54.6051544);

	public static final Angle FIXED_SHOT_ANGLE = Degrees.of(0);
	public static final AngularVelocity FIXED_SHOT_SPEED = RotationsPerSecond.of(65);

	//TODO: Change standby values to actual values after testing is done
	public static final Angle STANDBY_ANGLE = Degrees.of(0);
	public static final AngularVelocity STANDBY_SPEED = RotationsPerSecond.of(65); //TODO: Change standby values to good values

	//TODO: Change later if we actually add a table to do alliance shots
	public static final Angle ALLIANCE_SHOT_ANGLE = Degrees.of(45);
	public static final AngularVelocity ALLIANCE_SHOT_SPEED = RotationsPerSecond.of(65);

	// Numerical constants (moved from magic literals)
	public static final double SOLVER_EPSILON = 1e-6;
	public static final int SOLVER_ITERATIONS = 4;

	public static final double ZEROING_SPEED = -0.3;

	// Tolerances
	public static final double WHEEL_VELOCITY_TOLERANCE = 50.0; // in RotationsPerSecond units
	public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 0.01; // degrees

	// Simulation / physical defaults
	public static final double FLYWHEEL_MOI = 0.00117056; // Roughly accurate for flywheel
	public static final double FLYWHEEL_GEARING = 1.0; // Also roughly accurate
	public static final double HOOD_MOI = 0.0001;
	public static final double HOOD_GEARING = 75; //100/3 previously,
	public static final double HOOD_ARM_LENGTH_METERS = 0.2;

	// State defaults
	public static final AngularVelocity REVERSE_WHEEL_SPEED = RotationsPerSecond.of(-60);

	public static final int LEFT_SHOOTER_MOTOR_ID = 37;
	public static final int RIGHT_SHOOTER_MOTOR_ID = 38;
	public static final int HOOD_MOTOR_ID = 39;

	public static final int LIMIT_SWITCH_PORT = 9;
	public static final int BEAM_BREAK_PORT = 0;

	public static final Supplier<PIDController> HOOD_PID = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(0.057, 0.008, 0.0008); //.0384 good alr
			case SIM -> new PIDController(0.04, 0, 0.001); // Tuned in sim
			case TESTING -> new PIDController(0, 0, 0);
		};
	public static final Supplier<PIDController> HOOD_DOWN_PID = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(0.03, 0.0002, 0.0); //.0384 good alr
			case SIM -> new PIDController(0.04, 0, 0.001); // Tuned in sim
			case TESTING -> new PIDController(0, 0, 0);
		};
	public static final Supplier<PIDController> WHEEL_PID = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(0.5, 0, 0);
			case SIM -> new PIDController(0.0077, 0, 0);
			case TESTING -> new PIDController(0.1, 0, 0);
		};
	public static final Supplier<SimpleMotorFeedforward> WHEEL_FEEDFORWARD = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new SimpleMotorFeedforward(0.35, 0.125, 0.12);
			case SIM -> new SimpleMotorFeedforward(0.11, 0.1, 0.0);
			case TESTING -> new SimpleMotorFeedforward(0.1, 0.01, 0.001);
		};

	// Placeholder positions; replace with real field measurements, Define based on alliance side
	public static final Transform3d ROBOT_TO_SHOOTER = new Transform3d(-0.2270125, -0.119366, 19, new Rotation3d(0, 0, Math.PI / 2));
	public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.625, 4.08, Rotation2d.kZero);
	public static final Pose2d RED_HUB_POSE = new Pose2d(11.92, 4.08, Rotation2d.kZero);

	// Shot sample data for lookup tables (placeholder/example values)
	public static record ShotSampleData(double distanceMeters, Angle hoodAngle, AngularVelocity flywheelSpeed, double timeOfFlightSeconds) {}

	// Important positions and transforms
	public static final Distance TRENCH_RADIUS = Meters.of(1.0); // radius around trench where hood is forced down
}
