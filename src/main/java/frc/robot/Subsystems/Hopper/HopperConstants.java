package frc.robot.Subsystems.Hopper;

public class HopperConstants {

	//not real values atm
	public static final double SPIN_VELOCITY = -0.15;
	public static final double OFF_VELOCITY = 0;

	public static final double SPIN_KICK_VELOCITY = -1;
	public static final double SPIN_KICK_VELOCITY_2 = -1;

	public static final String SUBSYSTEM_NAME = "Hopper";

	public static final int SPINDEXER_MOTOR_ID = 33;
	public static final int KICKER_MOTOR_ID = 34;
	public static final int KICKER_MOTOR_2_ID = 50;

	public static class Sim {

		public static final double MOTOR_MOI = 0.00001;
		public static final int NUM_MOTORS = 1;
	}
}
