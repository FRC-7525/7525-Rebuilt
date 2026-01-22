package frc.robot.Subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystems.Hopper.HopperConstants.Sim;

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
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.motorVelocityRPS = spindexerSim.getAngularVelocity().in(RotationsPerSecond);
		outputs.targetVelocity = targetVelocity;

		spindexerSimState.setRotorVelocity(spindexerSim.getAngularVelocity());
	}

	@Override
	public void setTargetVelocity(double targetVelocity) {
		this.targetVelocity = targetVelocity;
		spindexerMotor.set(targetVelocity);
	}
}
