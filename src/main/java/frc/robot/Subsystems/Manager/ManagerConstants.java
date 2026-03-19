package frc.robot.Subsystems.Manager;


public class ManagerConstants {

	public static final String SUBSYSTEM_NAME = "Manager";

	public static final GameStates[] ALLIANCE_WON_AUTONOMOUS = {GameStates.TRANSITION_SHIFT, GameStates.HUB_NOT_ACTIVE, GameStates.HUB_ACTIVE, GameStates.HUB_NOT_ACTIVE, GameStates.HUB_ACTIVE, GameStates.ENDGAME};
	public static final GameStates[] ALLIANCE_LOST_AUTONOMOUS = {GameStates.TRANSITION_SHIFT, GameStates.HUB_ACTIVE, GameStates.HUB_NOT_ACTIVE, GameStates.HUB_ACTIVE, GameStates.HUB_NOT_ACTIVE, GameStates.ENDGAME};
	public static final GameStates[] UNKNOWN_ALLIANCE_WON = {GameStates.UNKNOWN};
}
