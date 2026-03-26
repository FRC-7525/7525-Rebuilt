package frc.robot.Subsystems.LEDs;

import java.util.function.Supplier;

import org.team7525.subsystem.SubsystemStates;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDStates implements SubsystemStates {
    IDLE("Idle", () -> LEDPattern.solid(Color.kRed));

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
