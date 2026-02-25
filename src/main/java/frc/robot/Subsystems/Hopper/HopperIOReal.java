package frc.robot.Subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Hopper.HopperConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;

public class HopperIOReal implements HopperIO {

	protected TalonFX spindexerMotor;
	protected TalonFX kickerMotor;
	protected TalonFX kickerMotor2;
	double targetSpinVelocity;
	double targetKickerVelocity;
	double targetKickerVelocity2;

	public HopperIOReal() {
		spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
		kickerMotor = new TalonFX(KICKER_MOTOR_ID);
		kickerMotor2 = new TalonFX(KICKER2_MOTOR_ID);
	}

	@Override
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.spinVelocityRPS = spindexerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.targetSpinVelocity = targetSpinVelocity;
		outputs.targetKickVelocity = targetKickerVelocity;
		outputs.targetKickVelocity2 = targetKickerVelocity2;
		double k1 = kickerMotor.getVelocity().getValue().in(RotationsPerSecond);
		double k2 = kickerMotor2.getVelocity().getValue().in(RotationsPerSecond);
		outputs.kickVelocityRPS1 = k1;
		outputs.kickVelocityRPS2 = k2;
	}

	@Override
	public void setTargetSpinVelocity(double targetSpinVelocity) {
		if (GlobalConstants.TESTING) {
			targetSpinVelocity = SmartDashboard.getNumber("Hopper Target Spin Velocity", targetSpinVelocity);
			SmartDashboard.putNumber("Hopper Target Spin Velocity", targetSpinVelocity);
		}
		this.targetSpinVelocity = targetSpinVelocity;
		spindexerMotor.set(targetSpinVelocity);
	}

	@Override
	public void setTargetKickVelocity(double targetKickVelocity) {
		this.targetKickerVelocity = targetKickVelocity;
		this.targetKickerVelocity2 = targetKickVelocity;
		kickerMotor.set(targetKickVelocity);
		kickerMotor2.set(targetKickVelocity);
	}

	@Override
	public void setTargetKickVelocity(double velocity1, double velocity2) {
		this.targetKickerVelocity = velocity1;
		this.targetKickerVelocity2 = velocity2;
		if (GlobalConstants.TESTING) {
			targetKickerVelocity = SmartDashboard.getNumber("Hopper Target Kick Velocity 1", targetKickerVelocity);
			targetKickerVelocity2 = SmartDashboard.getNumber("Hopper Target Kick Velocity 2", targetKickerVelocity2);
			SmartDashboard.putNumber("Hopper Target Kick Velocity 1", targetKickerVelocity);
			SmartDashboard.putNumber("Hopper Target Kick Velocity 2", targetKickerVelocity2);
		}
		kickerMotor.set(velocity1);
		kickerMotor2.set(velocity2);
	}
}
