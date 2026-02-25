package frc.robot.Subsystems.Hopper;

import org.team7525.subsystem.SubsystemStates;

public enum HopperStates implements SubsystemStates {
	IDLE("Idle", HopperConstants.OFF_VELOCITY, HopperConstants.OFF_VELOCITY, HopperConstants.OFF_VELOCITY),
	SPINDEXING("Spindexing", HopperConstants.SPIN_VELOCITY, HopperConstants.SPIN_KICK_VELOCITY, HopperConstants.SPIN_KICK_VELOCITY2);


	private String stateString;
	private double spinVelocity;
	private double kickVelocity;
	private double kickVelocity2;

	HopperStates(String stateString, double spinVelocity, double kickVelocity, double kickVelocity2) {
		this.stateString = stateString;
		this.spinVelocity = spinVelocity;
		this.kickVelocity = kickVelocity;
		this.kickVelocity2 = kickVelocity2;
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

	public double getKickVelocity2() {
		return kickVelocity2;
	}
}
