package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs; //need to add via phoenix tuner

public class IntakeConstants {

	public static final String SUBSYSTEM_NAME = "Intake";

	public static final double GEARING = 1;
	public static final double LINEAR_GEARING = 10.0; // figure out on CAD

	public static final double SET_TO_VOLTS_CF = 12;

	public static final Slot0Configs LINEAR_SLOT_0_CONFIGS = new Slot0Configs().withKP(0.5).withKI(0).withKD(0);

	// States
	public static final double INTAKE_IN_POS = 0.0;
	public static final double INTAKE_OUT_POS = 20.0; // Figure out on CAD

	public static final double SPIN_SPEED_INTAKE = 0.6; // probably too slow but wtv

	public static class Real {

		public static final int SPIN_MOTOR_ID = 20; //make real
		public static final int LINEAR_ACTUATOR_ID = 21;
	}

	public static class Sim {

		public static final int NUM_MOTORS = 1;
		public static final double MOTOR_MOI = 0.00001;
		public static final double LINEAR_KP = 10.0; // Tune in sim
	}
}
