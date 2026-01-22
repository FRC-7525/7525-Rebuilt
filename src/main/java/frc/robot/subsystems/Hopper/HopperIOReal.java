package frc.robot.Subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Hopper.HopperConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class HopperIOReal implements HopperIO {

	protected TalonFX spindexerMotor;
	protected TalonFX kickerMotor;
	double targetSpinVelocity;
	double targetKickerVelocity;

	public HopperIOReal() {
		spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
		kickerMotor = new TalonFX(KICKER_MOTOR_ID);
	}

	@Override
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.spinVelocityRPS = spindexerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.targetSpinVelocity = targetSpinVelocity;
		outputs.targetKickVelocity = targetKickerVelocity;
		outputs.kickVelocityRPS = kickerMotor.getVelocity().getValue().in(RotationsPerSecond);
	}

	@Override
	public void setTargetSpinVelocity(double targetSpinVelocity) {
		this.targetSpinVelocity = targetSpinVelocity;
		spindexerMotor.set(targetSpinVelocity);
	}

	@Override
	public void setTargetKickVelocity(double targetKickVelocity) {
		this.targetKickerVelocity = targetKickVelocity;
		kickerMotor.set(targetKickVelocity);
	}
}