package frc.robot.Subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Hopper.HopperConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class HopperIOReal implements HopperIO {

	protected TalonFX spindexerMotor;
	protected TalonFX kickerMotor;
	protected TalonFX kickerMotor2;
	double targetSpinVelocity;
	double targetKickerVelocity;

	public HopperIOReal() {
		spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
		kickerMotor = new TalonFX(KICKER_MOTOR_ID);
		kickerMotor2 = new TalonFX(KICKER_MOTOR_2_ID);
		kickerMotor2.setControl(new Follower(KICKER_MOTOR_ID, MotorAlignmentValue.Aligned));
	}

	@Override
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.spinVelocityRPS = spindexerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.kickVelocityRPS = kickerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.targetSpinVelocity = targetSpinVelocity;
		outputs.targetKickVelocity = targetKickerVelocity;
	}

	@Override
	public void setTargetSpinVelocity(double targetSpinVelocity) {
		this.targetSpinVelocity = targetSpinVelocity;
		spindexerMotor.set(targetSpinVelocity);
	}

	@Override
	public void setTargetKickerVelocity(double targetKickVelocity) {
		this.targetKickerVelocity = targetKickVelocity;
		kickerMotor.set(targetKickVelocity);
	}
}
