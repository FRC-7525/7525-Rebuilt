package frc.robot.Subsystems.Hopper;

public interface HopperIO {
	public class HopperIOOutputs {

		double targetVelocity;
		double motorVelocityRPS;
	}

	public void updateOutputs(HopperIOOutputs outputs);

	public void setTargetVelocity(double velocity);
}
