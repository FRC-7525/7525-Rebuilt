package frc.robot.Subsystems.Hopper;

public interface HopperIO {
	public class HopperIOOutputs {

		double targetSpinVelocity;
		double targetKickVelocity;
		double spinVelocityRPS;
		double kickVelocityRPS;
		double spindexerCurrent;
		double kickerCurrent;
	}

	public void updateOutputs(HopperIOOutputs outputs);

	public void setTargetSpinVelocity(double velocity);

	public void setTargetKickerVelocity(double velocity);
}
