package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.FieldConstants.*;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOOutputs;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Shooter extends Subsystem<ShooterStates> {

	private static Shooter instance;
	private final ShooterIO io;
	private ShooterIOOutputs outputs;
	private ShooterStates cache;
	private DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_PORT);
	private boolean previousBBValue = true;
	private int numBallsShot = 0;
	private boolean trenchProtection = true; // Whether to force the hood down in the trench
	private Timer bpsTime = new Timer();

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

	public void toggleTrenchProtection() {
		trenchProtection = !trenchProtection;
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
			if (
				trenchProtection &&
				(Drive.getInstance().getPose().relativeTo(TRENCH_POSE_LEFT_BLUE).getTranslation().getNorm() < TRENCH_RADIUS.in(Meters) ||
					Drive.getInstance().getPose().relativeTo(TRENCH_POSE_RIGHT_BLUE).getTranslation().getNorm() < TRENCH_RADIUS.in(Meters) ||
					Drive.getInstance().getPose().relativeTo(TRENCH_POSE_LEFT_RED).getTranslation().getNorm() < TRENCH_RADIUS.in(Meters) ||
					Drive.getInstance().getPose().relativeTo(TRENCH_POSE_RIGHT_RED).getTranslation().getNorm() < TRENCH_RADIUS.in(Meters))
			) {
				io.setHoodAngle(ShooterStates.IDLE.getHoodAngle());
			} else {
				io.setHoodAngle(getState().getHoodAngle());
			}
			io.setWheelVelocity(getState().getWheelVelocity());
			io.logOutputs(outputs);

			//TODO: Remove this stuff later when testing is done
			if (getState() == ShooterStates.SHOOT_FIXED) {
				if (previousBBValue != beamBreak.get()) {
					if (previousBBValue) {
						if (numBallsShot == 0) bpsTime.start();
						numBallsShot++;
					}
					previousBBValue = beamBreak.get();
				}
			} else {
				numBallsShot = 0;
				bpsTime.stop();
				bpsTime.reset();
			}
		} else if (io.zeroHoodMotor()) setState(cache);

		Logger.recordOutput(SUBSYSTEM_NAME + "/LeftWheelVelocity", outputs.leftWheelVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/RightWheelVelocity", outputs.rightWheelVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/WheelSetpoint", outputs.wheelSetpoint.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodAngle", outputs.hoodAngle.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodSetpoint", outputs.hoodSetpoint.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/state", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/ReadyToShoot", readyToShoot());
		Logger.recordOutput(SUBSYSTEM_NAME + "/BallsPerSecond", numBallsShot / bpsTime.get()); //TODO: Remove later when testing is done
		Logger.recordOutput(SUBSYSTEM_NAME + "/NumBallsShot", numBallsShot);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LeftWheelCurrent", outputs.leftWheelCurrent);
		Logger.recordOutput(SUBSYSTEM_NAME + "/RightWheelCurrent", outputs.rightWheelCurrent);
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodCurrent", outputs.hoodCurrent);
	}

	public boolean readyToShoot() {
		return io.atHoodAngleSetpoint() && io.atWheelVelocitySetpoint();
	}

	@Override
	protected void stateExit() {
		cache = getState();
	}

	public AngularVelocity getWheelSetpoint() {
		return getState().getWheelVelocity();
	}

	public AngularVelocity getWheelVelocity() {
		return io.getWheelVelocity();
	}

	public List<TalonFX> getShooterMotors() {
		return io.getShooterMotors();
	}

	public TalonFX getHoodMotor() {
		return io.getHoodMotor();
	}
}
