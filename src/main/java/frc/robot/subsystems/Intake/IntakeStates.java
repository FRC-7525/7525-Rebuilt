package frc.robot.Subsystems.Intake;

import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import edu.wpi.first.units.measure.Angle;
import org.team7525.subsystem.SubsystemStates;

public enum IntakeStates implements SubsystemStates {
	IN("In", INTAKE_IN_POS, 0),
	OUT("Out", INTAKE_OUT_POS, 0),
	INTAKE("Intake", INTAKE_OUT_POS, SPIN_SPEED_INTAKE),
	AGITATING("Agitating", INTAKE_AGITATING_IN_POS, SPIN_SPEED_INTAKE);

	private String stateString;
	public final Angle angularPos; //force units
	public final double spinSpeed;

	IntakeStates(String stateString, Angle angularPos, double spinSpeed) {
		this.stateString = stateString;
		this.angularPos = angularPos;
		this.spinSpeed = spinSpeed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public Angle getAngle() {
		return angularPos;
	}

	public double getSpinSpeed() {
		return spinSpeed;
	}
}
