package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class GlobalConstants {

	public static final int VOLTS = 12;

	public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);
	public static final double SIMULATION_PERIOD = 0.02;
	public static final Mass ROBOT_MASS = Pounds.of(135);

	public static final Field2d FIELD = new Field2d();

	public enum RobotMode {
		REAL,
		TESTING,
		SIM,
	}

	public static final RobotMode ROBOT_MODE = "Crash".equals(System.getenv("CI_NAME")) || !Robot.isReal() ? RobotMode.SIM : RobotMode.REAL;

	public static class Controllers {
		public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
		public static final XboxController OPERATOR_CONTROLLER = new XboxController(1);
		public static final XboxController TEST_CONTROLLER = new XboxController(4);

		// NOTE: Set to 0.1 on trash controllers
		public static final double DEADBAND = 0.1;
		public static final double TRIGGERS_REGISTER_POINT = 0.5;

		/**
		 * Apply the configured deadband to a controller axis value.
		 * Returns 0.0 when the absolute value is below DEADBAND, otherwise returns the original value.
		 *
		 * @param value axis value in range [-1, 1]
		 * @return value with deadband applied
		 */
		public static double applyDeadband(double value) {
			return Math.abs(value) < DEADBAND ? 0.0 : value;
		}
	}
}
