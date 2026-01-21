package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class GlobalConstants {
	public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);
	public static final double SIMULATION_PERIOD = 0.02;
	public static final Field2d FIELD = new Field2d();

	public enum RobotMode {
		REAL,
		TESTING,
		SIM,
	}

	public static final RobotMode ROBOT_MODE = "Crash".equals(System.getenv("CI_NAME")) || !Robot.isReal() ? RobotMode.SIM : RobotMode.REAL;

	public static class Controllers {
		public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
		public static final double DEADBAND = 0.01;
		public static final double TRIGGERS_REGISTER_POINT = 0.5;
	}
}