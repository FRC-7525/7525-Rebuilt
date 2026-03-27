package frc.robot.Subsystems.Hopper;

import com.ctre.phoenix6.hardware.TalonFX;

public interface HopperIO {
	public class HopperIOOutputs {

		double targetSpinVelocity;
		double targetKickVelocity;
		double spinVelocityRPS;
		double kickVelocityRPS;
		double spindexerCurrent;
		double kicker1Current;
		double kicker2Current;
	}

	public void updateOutputs(HopperIOOutputs outputs);

	public void setTargetSpinVelocity(double velocity);

	public void setTargetKickerVelocity(double velocity);

	public TalonFX getSpinMotor();

	public TalonFX getKickerMotor1();

	public TalonFX getKickerMotor2();
}
