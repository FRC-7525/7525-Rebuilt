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
	private final TalonFXSimState kicker2SimState;
	private final DCMotorSim spindexerSim;
	private final DCMotorSim kickerSim;
	private final DCMotorSim kicker2Sim;

	double targetSpinVelocity;
	double targetKickVelocity;
	double targetKickVelocity2;

	public HopperIOSim() {
		spindexerSimState = new TalonFXSimState(spindexerMotor);
		spindexerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));

		kickerSimState = new TalonFXSimState(kickerMotor);
		kickerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));

		kicker2SimState = new TalonFXSimState(kickerMotor2);
		kicker2Sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Sim.MOTOR_MOI, 1), DCMotor.getFalcon500(1));
	}

	@Override
	public void updateOutputs(HopperIOOutputs outputs) {
		outputs.spinVelocityRPS = spindexerSim.getAngularVelocity().in(RotationsPerSecond);
		outputs.targetSpinVelocity = targetSpinVelocity;
		outputs.targetKickVelocity = targetKickVelocity;
		outputs.targetKickVelocity2 = targetKickVelocity2;
		double k1 = kickerSim.getAngularVelocity().in(RotationsPerSecond);
		double k2 = kicker2Sim.getAngularVelocity().in(RotationsPerSecond);
		outputs.kickVelocityRPS1 = k1;
		outputs.kickVelocityRPS2 = k2;

		spindexerSimState.setRotorVelocity(spindexerSim.getAngularVelocity());
		kickerSimState.setRotorVelocity(kickerSim.getAngularVelocity());
		kicker2SimState.setRotorVelocity(kicker2Sim.getAngularVelocity());
	}

	@Override
	public void setTargetSpinVelocity(double targetSpinVelocity) {
		this.targetSpinVelocity = targetSpinVelocity;
		spindexerMotor.set(targetSpinVelocity);
	}

	@Override
	public void setTargetKickVelocity(double targetKickVelocity) {
		this.targetKickVelocity = targetKickVelocity;
		this.targetKickVelocity2 = targetKickVelocity;
		kickerMotor.set(targetKickVelocity);
		kickerMotor2.set(targetKickVelocity);
	}

	@Override
	public void setTargetKickVelocity(double velocity1, double velocity2) {
		this.targetKickVelocity = velocity1;
		this.targetKickVelocity2 = velocity2;
		kickerMotor.set(velocity1);
		kickerMotor2.set(velocity2);
	}
}
