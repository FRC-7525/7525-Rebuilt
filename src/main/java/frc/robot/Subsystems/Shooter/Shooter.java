package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOOutputs;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Shooter extends Subsystem<ShooterStates> {

	private static Shooter instance;
	private final ShooterIO io;
	private ShooterIOOutputs outputs;
	private ShooterStates cache;

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
		super(SUBSYSTEM_NAME, ShooterStates.IDLE);
		this.io = io;
		outputs = new ShooterIOOutputs();
		cache = ShooterStates.IDLE;
	}

	@Override
	public void runState() {
		if (getState() != ShooterStates.ZEROING) {
			io.setHoodAngle(getState().getHoodAngle());
			io.setWheelVelocity(getState().getWheelVelocity());
			io.logOutputs(outputs);
		} else if (io.zeroHoodMotor()) setState(cache);

		Logger.recordOutput(SUBSYSTEM_NAME + "/LeftWheelVelocity", outputs.leftWheelVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/RightWheelVelocity", outputs.rightWheelVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/WheelSetpoint", outputs.wheelSetpoint.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodAngle", outputs.hoodAngle.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodSetpoint", outputs.hoodSetpoint.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/state", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/ReadyToShoot", readyToShoot());
	}

	public boolean readyToShoot() {
		return io.atHoodAngleSetpoint() && io.atWheelVelocitySetpoint();
	}

	@Override
	protected void stateExit() {
		cache = getState();
	}
}
