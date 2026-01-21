package frc.robot.subsystems.Intake;

import static frc.robot.subsystems.Intake.IntakeConstants.*;

import org.team7525.subsystem.SubsystemStates;

public enum IntakeStates implements SubsystemStates {
	IN("In", INTAKE_IN_POS, 0.0),
	OUT("Out", INTAKE_OUT_POS, 0.0),
	INTAKE("Intake", INTAKE_OUT_POS, SPIN_SPEED_INTAKE);

	private String stateString;
	public final double linearPos;
	public final double spinSpeed;

	IntakeStates(String stateString, double linearPos, double spinSpeed) {
		this.stateString = stateString;
		this.linearPos = linearPos;
		this.spinSpeed = spinSpeed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}