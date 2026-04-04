package frc.robot.Subsystems.LEDs;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.LEDs.LEDsConstants.*;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Subsystems.Shooter.Shooter;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum LEDStates implements SubsystemStates {
	DISABLED("Disabled", () -> DISABLED_PATTERN),
	IDLE("Idle", () -> IDLE_PATTERN),
	INTAKING("Intaking", () -> INTAKING_PATTERN),
	WINDING_UP("Winding Up", () -> Math.abs(Shooter.getInstance().getWheelVelocity().minus(Shooter.getInstance().getWheelSetpoint()).in(RotationsPerSecond)) <= SETPOINT_DEVIATION ? WOUND_UP_PATTERN : WINDING_UP_PATTERN),
	SHOOTING("Shooting", () -> SHOOTING_PATTERN),
	SHUTTLING("Shuttling", () -> SHUTTLING_PATTERN);

	private final String stateName;
	private final LEDPattern pattern;

	LEDStates(String stateName, Supplier<LEDPattern> ledPattern) {
		this.stateName = stateName;
		this.pattern = ledPattern.get();
	}

	@Override
	public String getStateString() {
		return this.stateName;
	}

	public LEDPattern getPattern() {
		return this.pattern;
	}
}
