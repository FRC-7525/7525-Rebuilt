package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.GlobalConstants;
import java.util.function.Supplier;

public class ClimberConstants {

	public static final String SUBSYSTEM_NAME = "Climber";
	public static final int CLIMBER_MOTOR_ID = 30;
	public static final Supplier<PIDController> CLIMB_PID = () ->
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL -> new PIDController(1.0, 0.0, 0.0);
			case SIM -> new PIDController(0.05, 0.0, 0.0);
			case TESTING -> new PIDController(1.0, 0.0, 0.0);
		};
	public static final Angle CLIMB_POSITION_TOLERANCE = Rotations.of(0.01);
	public static final Angle IDLE_SETPOINT = Rotations.of(0.0);
	public static final Angle EXTEND_SETPOINT = Rotations.of(0.0833333333);
	public static final Angle RETRACT_SETPOINT = Rotations.of(-0.0138888889);
	public static final Angle HOLD_SETPOINT = Rotations.of(0.0013888889);
	public static final double CLIMBER_MOI = 0.05; // moment of inertia (placeholder)
	public static final double CLIMBER_GEARING = 100;
	public static final double CLIMBER_ARM_LENGTH_METERS = 0.1;

	public static final double CLIMBER_MASS = 1.0; // kg (placeholder)
	public static final double CLIMBER_RADIUS = 0.02; // meters (placeholder)
	public static final Distance DRUM_RADIUS = Meters.of(0.02); // meters (placeholder)
	public static final Distance DRUM_CIRCUMFERENCE = DRUM_RADIUS.times(Math.PI * 2);
	public static final double START_HEIGHT = 0.0; // meters (placeholder)
	public static final double END_HEIGHT = 0.1; // meters (placeholder)
}
