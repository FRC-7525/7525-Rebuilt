package frc.robot.Subsytems.Shooter;

import static frc.robot.Subsytems.Shooter.ShooterConstants.*;

import frc.robot.GlobalConstants;
import frc.robot.Subsytems.Shooter.ShooterIO.ShooterIOOutputs;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Shooter extends Subsystem<ShooterStates> {

	private static Shooter instance;
	private final ShooterIO io;
	private ShooterIOOutputs outputs;

	public static Shooter getInstance() {
		if (instance == null) {
			switch (GlobalConstants.ROBOT_MODE) {
				case REAL -> instance = new Shooter(new ShooterIOReal());
				case SIM -> instance = new Shooter(new ShooterIOSim());
				case TESTING -> instance = new Shooter(new ShooterIOReal());
			}
		}
		return instance;
	}

	private Shooter(ShooterIO io) {
		super(SUBSYSTEM_NAME, ShooterStates.OFF);
		this.io = io;
		outputs = new ShooterIOOutputs();
	}

	@Override
	public void runState() {
		io.setHoodAngle(getState().getHoodAngle());
		io.setWheelVelocity(getState().getWheelVelocity());
		io.logOutputs(outputs);

		Logger.recordOutput(SUBSYSTEM_NAME + "/LeftWheelVelocity", outputs.leftWheelVelocity);
		Logger.recordOutput(SUBSYSTEM_NAME + "/RightWheelVelocity", outputs.rightWheelVelocity);
		Logger.recordOutput(SUBSYSTEM_NAME + "/WheelSetpoint", outputs.wheelSetpoint);
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodAngle", outputs.hoodAngle);
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodSetpoint", outputs.hoodSetpoint);
		Logger.recordOutput(SUBSYSTEM_NAME + "/state", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/ReadyToShoot", readyToShoot());
	}

	public boolean readyToShoot() {
		return io.atHoodAngleSetpoint() && io.atWheelVelocitySetpoint();
	}
}
