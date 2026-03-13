package frc.robot.Subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Hopper.HopperConstants.*;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class HopperIOReal implements HopperIO {

	protected TalonFX spindexerMotor;
	protected TalonFX kickerMotor;
	protected TalonFX kickerMotor2;
	protected TalonFXConfiguration spindexerMotorConfig = new TalonFXConfiguration();
	protected TalonFXConfiguration kickerMotor1Config = new TalonFXConfiguration();
	protected TalonFXConfiguration kickerMotor2Config = new TalonFXConfiguration();
	double targetSpinVelocity;
	double targetKickerVelocity;

	public HopperIOReal() {
		spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
		kickerMotor = new TalonFX(KICKER_MOTOR_ID);
		kickerMotor2 = new TalonFX(KICKER_MOTOR_2_ID);
		kickerMotor2.setControl(new Follower(KICKER_MOTOR_ID, MotorAlignmentValue.Opposed));

		// spindexerMotorConfig.CurrentLimits.StatorCurrentLimit = 90;
		// kickerMotor1Config.CurrentLimits.StatorCurrentLimit = 90;
		// kickerMotor2Config.CurrentLimits.StatorCurrentLimit = 90;

		// spindexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
		// kickerMotor1Config.CurrentLimits.SupplyCurrentLimit = 30;
		// kickerMotor2Config.CurrentLimits.SupplyCurrentLimit = 10;

		spindexerMotor.getConfigurator().apply(spindexerMotorConfig);
		kickerMotor.getConfigurator().apply(kickerMotor1Config);
		kickerMotor2.getConfigurator().apply(kickerMotor2Config);
	}

	@Override
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.spinVelocityRPS = spindexerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.kickVelocityRPS = kickerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.targetSpinVelocity = targetSpinVelocity;
		outputs.targetKickVelocity = targetKickerVelocity;
		outputs.spindexerCurrent = spindexerMotor.getSupplyCurrent().getValueAsDouble();
		outputs.kicker1Current = kickerMotor.getSupplyCurrent().getValueAsDouble();
		outputs.kicker2Current = kickerMotor2.getSupplyCurrent().getValueAsDouble();
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

	public TalonFX getSpinMotor() {
		return spindexerMotor;
	}

	public TalonFX getKickerMotor1() {
		return kickerMotor;
	}

	public TalonFX getKickerMotor2() {
		return kickerMotor2;
	}
}
