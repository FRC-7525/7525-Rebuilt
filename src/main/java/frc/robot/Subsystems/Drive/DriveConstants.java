package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

public class DriveConstants {

	public static final double SIM_UPDATE_TIME = 0.004;

	public static final Distance WHEEL_BASE = Meters.of(0.5);

	public static final double MIN_SCALE_FACTOR = 0.1;

	public static final double SLOW_MODE_MULTIPLIER = 0.5;

	public static final double CLOSE_TO_ZERO = Math.pow(10, -4);

	public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(11.7);

	public static final AngularVelocity ANGULAR_VELOCITY_LIMIT = AngularVelocity.ofBaseUnits(180, DegreesPerSecond);

	public static final LinearAcceleration MAX_LINEAR_DECELERATION = MetersPerSecondPerSecond.of(11);
	public static final LinearAcceleration MAX_LINEAR_STOPPING_ACCELERATION = MetersPerSecondPerSecond.of(10);

	public static final String SUBSYSTEM_NAME = "Drive";

	// For zeroing on robot init
	public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
	public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);

	public static final Supplier<PIDController> X_AUTO_CONTROLLER = () -> // TODO: TUNE
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(5, 0, 0);
			case SIM -> new PIDController(5, 0, 0);
			default -> new PIDController(20, 1, 0);
		};

	public static final Supplier<PIDController> Y_AUTO_CONTROLLER = () -> // TODO: TUNE
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(5, 0, 0);
			case SIM -> new PIDController(5, 0, 0);
			default -> new PIDController(20, 1, 0);
		};

	public static final Supplier<PIDController> ANGLE_AUTO_CONTROLLER = () -> // TODO: TUNE
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(3, 0, 0);
			case SIM -> new PIDController(5, 0, 0);
			default -> new PIDController(20, 1, 0);
		};
	// Weird syntax because we have our own PIDConstants class (literally just the PP one :skull: copy pasted) so we can use it without installing PP Lib
	public static final PPHolonomicDriveController PATH_PLANNER_PID = new PPHolonomicDriveController(new com.pathplanner.lib.config.PIDConstants(5.0, 0, 0), new com.pathplanner.lib.config.PIDConstants(5.0, 0, 0));

	public static final Supplier<PIDController> SNAKE_DRIVE_CONTROLLER = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(3, 0, 0);
			case SIM -> new PIDController(10, 0, .01);
			default -> new PIDController(20, 1, 0);
		};
}
