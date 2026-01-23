package frc.robot.Subsystems.Climber;

import org.team7525.subsystem.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	IDLE("Idle",0),
    READY("Ready", 0),
	L1("L1", 0);

	private String stateString;
	private double climberSetpoint;
	

	ClimberStates(String stateString, double climberSetpoint) {
		this.stateString = stateString;
		this.climberSetpoint = climberSetpoint;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getClimberSetpoint() {
		return climberSetpoint;
	}
}
