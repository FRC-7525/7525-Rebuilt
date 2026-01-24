package frc.robot.Subsystems.Climber;

import edu.wpi.first.units.measure.Angle;
import org.team7525.subsystem.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	IDLE("IDLE", ClimberConstants.IDLE_SETPOINT),
	EXTEND("EXTEND", ClimberConstants.EXTEND_SETPOINT),
	RETRACT("RETRACT", ClimberConstants.RETRACT_SETPOINT),
	HOLD("HOLD", ClimberConstants.HOLD_SETPOINT);

	private final String stateString;
	private final Angle setpoint;

	ClimberStates(String stateString, Angle setpoint) {
		this.stateString = stateString;
		this.setpoint = setpoint;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public Angle getClimberSetpoint() {
		return setpoint;
	}
}
