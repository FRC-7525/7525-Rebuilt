package frc.robot.subsystems.Hopper;

import frc.robot.subsystems.Hopper.HopperConstants;
import org.team7525.subsystem.SubsystemStates;

public enum HopperStates implements SubsystemStates {
	IDLE("Idle", HopperConstants.OFF_VELOCITY),
	SPINDEXING("Expelling", HopperConstants.SPIN_VELOCITY);

	private String stateString;
	private double velocity;

	HopperStates(String stateString, double velocity) {
		this.stateString = stateString;
		this.velocity = velocity;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getVelocity() {
		return velocity;
	}
}
