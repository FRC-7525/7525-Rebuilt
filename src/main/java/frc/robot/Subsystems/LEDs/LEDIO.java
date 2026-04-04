package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;

public interface LEDIO {
	public abstract void setManagerPattern(LEDPattern pattern);

	public abstract void setDrivePattern(LEDPattern pattern);

	public abstract void setLEDData();
}
