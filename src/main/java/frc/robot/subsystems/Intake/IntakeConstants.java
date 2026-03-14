package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.GlobalConstants.ROBOT_MODE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;
import org.team7525.controlConstants.PIDConstants;

public class IntakeConstants {

	public static final String SUBSYSTEM_NAME = "Intake";

	public static final double GEARING = 96;
	public static final Supplier<PIDController> PIVOT_PID = () ->
		switch (ROBOT_MODE) {
			case SIM -> new PIDController(0.0, 0.0, 0.0);
			case REAL, TESTING -> new PIDController(0.05, 0.0, 0.0);
		};

	// States
	public static final Angle INTAKE_IN_POS = Degrees.of(0.0);
	public static final Angle INTAKE_OUT_POS = Degrees.of(-170);

	public static final Angle INTAKE_AGITATING_IN_POS = Degrees.of(-60);
	public static final Angle INTAKE_AGITATING_OUT_POS = Degrees.of(-120);

	public static final double SPIN_SPEED_INTAKE = -0.5;

	public static class Real {

		public static final int SPIN_MOTOR_ID = 35; //make real
		public static final int PIVOT_MOTOR_ID = 36;
	}

	public static class Sim {

		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.0001;
		public static final PIDConstants LINEAR_PID = new PIDConstants(5.5, 0.0, 0.008); // Tuned in sim	}
	}
}
