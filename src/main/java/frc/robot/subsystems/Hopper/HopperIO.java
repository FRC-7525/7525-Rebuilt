package frc.robot.subsystems.Hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
	@AutoLog
	public class HopperIOInputs {

		double inputVoltage;
		double targetVelocity;
		double motorVelocityRPS;
	}

	public void updateInput(HopperIOInputs inputs);

	public void setTargetVelocity(double velocity);
}
