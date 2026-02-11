package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOOutputs;
import frc.robot.Subsystems.Shooter.ShotSolver.ShotSolution;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Shooter extends Subsystem<ShooterStates> {

	private static Shooter instance;
	private final ShooterIO io;
	private ShooterIOOutputs outputs;
	private ShotSolution currentShotSolution;

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
	}

	@Override
	public void runState() {
		currentShotSolution = getState().getShotSolution();
		io.setHoodAngle(currentShotSolution != null ? currentShotSolution.getHoodAngle() : Degrees.of(0.0));
		io.setWheelVelocity(currentShotSolution != null ? currentShotSolution.getFlywheelSpeed() : RotationsPerSecond.of(0.0));
		io.logOutputs(outputs);
		Drive.getInstance().setSOTMTarget(currentShotSolution != null ? currentShotSolution.getVirtualTarget() : BLUE_HUB_POSE);

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

	public double getStateTime() {
		return super.getStateTime();
	}
}
