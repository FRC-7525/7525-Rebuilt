package frc.robot.Subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Hopper.HopperConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class HopperIOReal implements HopperIO {

	protected TalonFX spindexerMotor;
	protected TalonFX kickerMotor;
	protected TalonFX kickerMotor2;
	double targetSpinVelocity;
	double targetKickerVelocity;
	double targeKicker2Velociy;

	public HopperIOReal() {
		spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
		kickerMotor = new TalonFX(KICKER_MOTOR_ID);
		kickerMotor2 = new TalonFX(KICKER_MOTOR_2_ID);
	}

	@Override
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.spinVelocityRPS = spindexerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.targetSpinVelocity = targetSpinVelocity;
		outputs.targetKickVelocity = targetKickerVelocity;
		outputs.kickVelocityRPS = kickerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.targetKickVelocity2 = targeKicker2Velociy;
	}

	@Override
	public void setTargetSpinVelocity(double targetSpinVelocity) {
		this.targetSpinVelocity = targetSpinVelocity;
		spindexerMotor.set(targetSpinVelocity);
	}

	@Override
	public void setTargetKickVelocity(double targetKickVelocity, double targetKickVelocity2) {
		this.targetKickerVelocity = targetKickVelocity;
		this.targeKicker2Velociy = targetKickVelocity2;
		kickerMotor.set(targetKickVelocity);
		kickerMotor2.set(targetKickVelocity2);
	}
}
