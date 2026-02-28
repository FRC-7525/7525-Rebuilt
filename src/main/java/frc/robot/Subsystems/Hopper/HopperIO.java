package frc.robot.Subsystems.Hopper;

public interface HopperIO {
	public class HopperIOOutputs {

		double targetSpinVelocity;
		double targetKickVelocity;
		double targetKickVelocity2;
		double spinVelocityRPS;
		double kickVelocityRPS;
	}

	public void updateOutputs(HopperIOOutputs outputs);

	public void setTargetSpinVelocity(double velocity);

	public void setTargetKickVelocity(double velocity, double velocity2);
}
