package frc.robot.Subsystems.Hopper;

import static frc.robot.GlobalConstants.ROBOT_MODE;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import frc.robot.Subsystems.Hopper.HopperIO.HopperIOOutputs;

public class Hopper extends Subsystem<HopperStates> {

	private static Hopper instance;
	private final HopperIO io;
	private final HopperIOOutputs outputs;

	private Hopper() {
		super(HopperConstants.SUBSYSTEM_NAME, HopperStates.IDLE);
		outputs = new HopperIOOutputs();
		this.io = switch (ROBOT_MODE) {
			case REAL -> new HopperIOReal();
			case SIM -> new HopperIOSim();
			case TESTING -> new HopperIOReal();
		};
	}

	public static Hopper getInstance() {
		if (instance == null) {
			instance = new Hopper();
		}
		return instance;
	}

	@Override
	protected void runState() {
		io.setTargetVelocity(getState().getVelocity());
		io.updateOutputs(outputs);

		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/MotorVelocityRPS", outputs.motorVelocityRPS);
		Logger.recordOutput(HopperConstants.SUBSYSTEM_NAME + "/TargetVelocity", outputs.targetVelocity);
	}
}
