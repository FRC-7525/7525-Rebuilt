package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.Hopper.HopperConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Hopper.HopperConstants.Sim;

public class HopperIOSim extends HopperIOReal {

	private final TalonFXSimState spindexerSimState;
	private final DCMotorSim spindexerSim;

	double targetVelocity;

	public HopperIOSim() {
		spindexerSimState = new TalonFXSimState(spindexerMotor);
		spindexerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));

		targetVelocity = 0.0;
	}

	@Override
	public void updateInput(HopperIOInputs inputs) {
		inputs.motorVelocityRPS = spindexerSim.getAngularVelocity().in(RotationsPerSecond);
		inputs.targetVelocity = targetVelocity;

		spindexerSimState.setRotorVelocity(spindexerSim.getAngularVelocity());
	}

	@Override
	public void setTargetVelocity(double targetVelocity) {
		this.targetVelocity = targetVelocity;
		spindexerMotor.set(targetVelocity);
	}
}
