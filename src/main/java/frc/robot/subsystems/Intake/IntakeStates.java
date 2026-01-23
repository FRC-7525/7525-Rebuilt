package frc.robot.subsystems.Intake;

import static frc.robot.subsystems.Intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import org.team7525.subsystem.SubsystemStates;

public enum IntakeStates implements SubsystemStates {
	IN("In", Meters.of(INTAKE_IN_POS), RotationsPerSecond.of(0.0)),
	OUT("Out", Meters.of(INTAKE_OUT_POS), RotationsPerSecond.of(0.0)),
	INTAKE("Intake", Meters.of(INTAKE_OUT_POS), RotationsPerSecond.of(SPIN_SPEED_INTAKE));

	private String stateString;
	public final Distance linearPos;  //force units
	public final AngularVelocity spinSpeed;

	IntakeStates(String stateString, Distance linearPos, AngularVelocity spinSpeed) {
		this.stateString = stateString;
		this.linearPos = linearPos;
		this.spinSpeed = spinSpeed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}