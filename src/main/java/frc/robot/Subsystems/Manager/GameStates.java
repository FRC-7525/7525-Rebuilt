package frc.robot.Subsystems.Manager;

public enum GameStates {
	HUB_ACTIVE("ACTIVE"),
	HUB_NOT_ACTIVE("NOT ACTIVE"),
	TRANSITION_SHIFT("TRANSITION SHIFT"),
	ENDGAME("ENDGAME"),
	UNKNOWN("UNKNOWN");

	String stateString;

	GameStates(String stateString) {
		this.stateString = stateString;
	}

	public String getStateString() {
		return stateString;
	}
}
