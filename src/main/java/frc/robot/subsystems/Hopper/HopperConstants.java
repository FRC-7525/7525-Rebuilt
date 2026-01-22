package frc.robot.subsystems.Hopper;

public class HopperConstants {
    //not real values atm
    public static final double INTAKING_VELOCITY = 1;
	public static final double EXPELLING_VELOCITY = -1;
	public static final double OFF_VELOCITY = 0;

    public static final String SUBSYSTEM_NAME = "Hopper";
    
    public static final int SPINDEXER_MOTOR_ID = 0;

    public static class Sim {
        public static final double MOTOR_MOI = 0.00001;
        public static final int NUM_MOTORS = 1;
    }
}
