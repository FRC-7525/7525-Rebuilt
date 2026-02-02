package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs; //need to add via phoenix tuner
import org.team7525.controlConstants.PIDConstants;

public class IntakeConstants {

	public static final String SUBSYSTEM_NAME = "Intake";

	public static final double GEARING = 1;
	public static final double LINEAR_GEARING = 10.0; // figure out on CAD
	public static final double LINEAR_METERS_PER_ROTATION = 0.1; // Check CAD/ motor specs

	// States
	public static final double INTAKE_IN_POS = 0.0;
	public static final double INTAKE_OUT_POS = 0.2; // Meters

	public static final double SPIN_SPEED_INTAKE = 60.0; // RPS

	public static final double SPIN_kS = 0.0003; // Sim tuned
	public static final double SPIN_kV = 0.00934498; // Sim tuned
	public static final double SPIN_kA = 0.0; // Not needed for thuis prob

	public static final Slot0Configs LINEAR_SLOT_0_CONFIGS = new Slot0Configs().withKP(0.5).withKI(0).withKD(0);
	public static final Slot0Configs SPIN_SLOT_0_CONFIGS = new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKV(SPIN_kV);

	public static class Real {

		public static final int SPIN_MOTOR_ID = 35; //make real
		public static final int LINEAR_ACTUATOR_ID = 36;
	}

	public static class Sim {

		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.0001;
		public static final PIDConstants LINEAR_PID = new PIDConstants(5.5, 0.0, 0.008); // Tuned in sim	}
	}
}
