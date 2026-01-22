package frc.robot.Subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Hopper.HopperConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class HopperIOReal implements HopperIO {

	protected TalonFX spindexerMotor;
	double targetVelocity;

	public HopperIOReal() {
		spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
		targetVelocity = 0.0;
	}

	@Override
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.motorVelocityRPS = spindexerMotor.getVelocity().getValue().in(RotationsPerSecond);
		outputs.targetVelocity = targetVelocity;
	}

	@Override
	public void setTargetVelocity(double targetVelocity) {
		this.targetVelocity = targetVelocity;
		spindexerMotor.set(targetVelocity);
	}
}
