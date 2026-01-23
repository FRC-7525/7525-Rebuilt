package frc.robot.Subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystems.Hopper.HopperConstants.Sim;

public class HopperIOSim extends HopperIOReal {

	private final TalonFXSimState spindexerSimState;
	private final TalonFXSimState kickerSimState;
	private final DCMotorSim spindexerSim;
	private final DCMotorSim kickerSim;

	double targetSpinVelocity;
	double targetKickVelocity;

	public HopperIOSim() {
		spindexerSimState = new TalonFXSimState(spindexerMotor);
		spindexerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));

		kickerSimState = new TalonFXSimState(kickerMotor);
		kickerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));
	}

	@Override
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.spinVelocityRPS = spindexerSim.getAngularVelocity().in(RotationsPerSecond);
		outputs.targetSpinVelocity = targetSpinVelocity;
		outputs.targetKickVelocity = targetKickVelocity;
		outputs.kickVelocityRPS = kickerSim.getAngularVelocity().in(RotationsPerSecond);

		spindexerSimState.setRotorVelocity(spindexerSim.getAngularVelocity());
		kickerSimState.setRotorVelocity(kickerSim.getAngularVelocity());
	}

	@Override
	public void setTargetSpinVelocity(double targetSpinVelocity) {
		this.targetSpinVelocity = targetSpinVelocity;
		spindexerMotor.set(targetSpinVelocity);
	}

	@Override
	public void setTargetKickVelocity(double targetKickVelocity) {
		this.targetKickVelocity = targetKickVelocity;
		kickerMotor.set(targetKickVelocity);
	}
}
