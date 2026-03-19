package frc.robot.Subsystems.Manager;

public enum GameStates {
    HUB_ACTIVE("ACTIVE", 25),
    HUB_NOT_ACTIVE("NOT ACTIVE", 25),
    TRANSITION_SHIFT("TRANSITION SHIFT", 10),
    ENDGAME("ENDGAME", 30),
    UNKNOWN("UNKNOWN", 25);
    
    String stateString;
    double stateDuration;

    GameStates(String stateString, double stateDuration) {
        this.stateString = stateString;
        this.stateDuration = stateDuration;
    }

    public String getStateString() {
        return stateString;
    }

    public double getStateDuration() {
        return stateDuration;
    }

}
