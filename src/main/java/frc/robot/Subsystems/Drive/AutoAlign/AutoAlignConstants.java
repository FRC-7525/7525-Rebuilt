package frc.robot.Subsystems.Drive.AutoAlign;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.GlobalConstants;
import java.util.List;
import java.util.function.Supplier;
import org.team7525.autoAlign.RepulsorFieldPlanner.GuidedObstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.HorizontalObstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.Obstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.PointObstacle;
import org.team7525.autoAlign.RepulsorFieldPlanner.VerticalObstacle;

public final class AutoAlignConstants {

	public static final Distance ROBOT_RADIUS = Meters.of(1.001);

	public static final Angle MIN_HEADING_ANGLE = Degrees.of(-180);
	public static final Angle MAX_HEADING_ANGLE = Degrees.of(180);

	public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(15);
	public static final boolean USE_GOAL = true;

	// Sim
	public static final Distance DISTANCE_ERROR_MARGIN = Meters.of(0.0208);
	public static final Distance DISTANCE_ERROR_MARGIN2 = Meters.of(0.5);
	public static final Angle ANGLE_ERROR_MARGIN = Degrees.of(0.5); // was 0.05 radians

	public static final double TIMEOUT_DISTANCE_THRESHOLD = 0.05;
	public static final double TIMEOUT_THRESHOLD = 1;
	public static final Angle MAX_YAW_ERROR = Degrees.of(0.5);

	public static final double GOAL_STRENGTH = 0.1;
	static final double FIELD_LENGTH = 16.42;
	static final double FIELD_WIDTH = 8.16;

	public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(2);
	public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(1);

	public static final Supplier<PIDController> SHOOTER_YAW_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(0.1, 0, 0);
			case SIM -> new PIDController(1, 0, .01);
			default -> new PIDController(20, 1, 0);
		};
	public static final Angle SWITCH_DIST = Degrees.of(5); // When to switch from fast(below) to slow(above) controller
	public static final Supplier<PIDController> SHOOTER_YAW_CONTROLLER_FAST = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1, 0, 0);
			case SIM -> new PIDController(2, 0, .01);
			default -> new PIDController(20, 1, 0);
		};

	public static final Supplier<ProfiledPIDController> SCALED_FF_TRANSLATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new ProfiledPIDController(25, 0, 0, new TrapezoidProfile.Constraints(Units.feetToMeters(9), 7), 0.02);
			case SIM -> new ProfiledPIDController(20, 1, 0, new TrapezoidProfile.Constraints(Units.feetToMeters(9), 7), 0.02);
			default -> new ProfiledPIDController(20, 1, 0, new TrapezoidProfile.Constraints(Units.feetToMeters(9), 7), 0.02);
		};

	public static final Supplier<ProfiledPIDController> SCALED_FF_ROTATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new ProfiledPIDController(20, 0, 1, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2), 0.02);
			case SIM -> new ProfiledPIDController(20, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2), 0.02);
			default -> new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2), 0.02);
		};

	public static final Supplier<PIDController> REPULSOR_TRANSLATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(5, 0, 0);
			case SIM -> new PIDController(5, 0, 0);
			default -> new PIDController(1, 0, 0);
		};

	public static final Supplier<PIDController> REPULSOR_ROTATIONAL_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(5, 0, 0);
			case SIM -> new PIDController(20, 0, 0);
			default -> new PIDController(10, 0, 0);
		};

	// Aliance zones
	public static final PosePair BLUE_ALLIANCE_ZONE = new PosePair(new Pose2d(0, 0, Rotation2d.kZero), new Pose2d(4.625, FIELD_WIDTH, Rotation2d.kZero));
	public static final PosePair RED_ALLIANCE_ZONE = new PosePair(new Pose2d(11.92, 0, Rotation2d.kZero), new Pose2d(FIELD_LENGTH, FIELD_WIDTH, Rotation2d.kZero));

	// Aimlock Positions
	public static final PosePair AIMLOCK_LEFT_DEEP_POSES = new PosePair(new Pose2d(15.5, 2.000, Rotation2d.kZero), new Pose2d(1, 6, Rotation2d.kZero));
	public static final PosePair AIMLOCK_LEFT_SHALLOW_POSES = new PosePair(new Pose2d(13.5, 2.000, Rotation2d.kZero), new Pose2d(3, 6, Rotation2d.kZero));
	public static final PosePair AIMLOCK_RIGHT_DEEP_POSES = new PosePair(new Pose2d(15.5, 6.000, Rotation2d.kZero), new Pose2d(1, 2, Rotation2d.kZero));
	public static final PosePair AIMLOCK_RIGHT_SHALLOW_POSES = new PosePair(new Pose2d(13.5, 6.000, Rotation2d.kZero), new Pose2d(3, 2, Rotation2d.kZero));
	public static final PosePair NEUTRAL_POSES = new PosePair(new Pose2d(9.5, 4.08, Rotation2d.kZero), new Pose2d(7, 2, Rotation2d.kZero));

	// AA Poses
	public static final PosePair TOWER_LEFT = new PosePair(new Pose2d(14.946, 4.898, new Rotation2d(180)), new Pose2d(1.554, 3.102, new Rotation2d(0)));
	public static final PosePair TOWER_RIGHT = new PosePair(new Pose2d(14.946, 4.119, new Rotation2d(180)), new Pose2d(1.554, 3.881, new Rotation2d(0)));

	public static final class Obstacles {

		public static final List<Obstacle> FIELD_OBSTACLES = List.of(new GuidedObstacle(1.0, true, Meters.of(0.5), new Translation2d(4.6, 2.625)), new GuidedObstacle(1.0, true, Meters.of(0.5), new Translation2d(4.6, 4)), new PointObstacle(1.0, true, Meters.of(0.5), new Translation2d(4.6, 5.375)));

		public static final List<Obstacle> WALLS = List.of(
			new HorizontalObstacle(0.5, true, 0, Meters.of(1)),
			new HorizontalObstacle(0.5, true, FIELD_WIDTH, Meters.of(1)),
			new VerticalObstacle(0.5, true, 0, Meters.of(1)),
			new VerticalObstacle(0.5, true, FIELD_LENGTH, Meters.of(1)),
			new VerticalObstacle(0.5, true, FIELD_LENGTH, Meters.of(FIELD_LENGTH))
		);
	}
}
