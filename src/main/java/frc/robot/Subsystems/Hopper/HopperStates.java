package frc.robot.Subsystems.Hopper;

import org.team7525.subsystem.SubsystemStates;

public enum HopperStates implements SubsystemStates {
	IDLE("Idle", HopperConstants.OFF_VELOCITY, HopperConstants.OFF_VELOCITY),
	SPINDEXING("Spindexing", HopperConstants.SPIN_KICK_VELOCITY, HopperConstants.SPIN_VELOCITY);

	private String stateString;
	private double spinVelocity;
	private double kickVelocity;

	HopperStates(String stateString, double spinVelocity, double kickVelocity) {
		this.stateString = stateString;
		this.spinVelocity = spinVelocity;
		this.kickVelocity = kickVelocity;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getSpinVelocity() {
		return spinVelocity;
	}

	public double getKickVelocity() {
		return kickVelocity;
	}
}
