package frc.robot.Subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Subsystems.LEDs.LEDsConstants.DISABLED_BREATH_PERIOD;
import static frc.robot.Subsystems.LEDs.LEDsConstants.IDLE_SCROLL_SPEED;
import static frc.robot.Subsystems.LEDs.LEDsConstants.INTAKING_SCROLL_SPEED;
import static frc.robot.Subsystems.LEDs.LEDsConstants.PIONEERS_BLUE;
import static frc.robot.Subsystems.LEDs.LEDsConstants.PIONEERS_ORANGE;

import java.util.Map;
import java.util.function.Supplier;

import org.team7525.subsystem.SubsystemStates;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDStates implements SubsystemStates {
    DISABLED("Disabled", LEDPattern.solid(Color.kRed).breathe(Seconds.of(DISABLED_BREATH_PERIOD))),
    IDLE("Idle", LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.fromHSV(PIONEERS_BLUE.hue, PIONEERS_BLUE.sat, PIONEERS_BLUE.val), Color.fromHSV(PIONEERS_ORANGE.hue, PIONEERS_ORANGE.sat, PIONEERS_ORANGE.val)).scrollAtRelativeSpeed(Percent.per(Second).of(IDLE_SCROLL_SPEED))),
    INTAKING("Intaking", LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, Color.kBlack, Color.kSeaGreen).scrollAtRelativeSpeed(Percent.per(Second).of(INTAKING_SCROLL_SPEED)))
    WINDING_UP("WINDING_UP", );

    private final String stateName;
    private final LEDPattern pattern;

    LEDStates(String stateName, LEDPattern ledPattern) {
        this.stateName = stateName;
        this.pattern = ledPattern;
    }

    @Override
	public String getStateString() {
		return this.stateName;
	}

    public LEDPattern getPattern() {
        return this.pattern;
    }
}
