package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOOutputs;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Shooter extends Subsystem<ShooterStates> {

	private static Shooter instance;
	private final ShooterIO io;
	private ShooterIOOutputs outputs;
	private ShooterStates cache;
	public LinearFilter smoothFilter;
	private DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_PORT);
	private boolean previousBBValue = true;
	private int numBallsShot = 0;

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
		smoothFilter = LinearFilter.movingAverage(5);
	}

	@Override
	public void runState() {
		smoothFilter.calculate(io.getShooterMotors().get(0).getVelocity().getValueAsDouble());
		if (getState() != ShooterStates.ZEROING) {
			io.setHoodAngle(getState().getHoodAngle());
			io.setWheelVelocity(getState().getWheelVelocity());
			io.logOutputs(outputs);

			//TODO: Remove this stuff later when testing is done
			if (getState() == ShooterStates.SHOOT_FIXED) {
				if (previousBBValue != beamBreak.get()) {
					if (previousBBValue) numBallsShot++;
					previousBBValue = beamBreak.get();
				}
			} else numBallsShot = 0;
		} else if (io.zeroHoodMotor()) setState(cache);

		Logger.recordOutput(SUBSYSTEM_NAME + "/LeftWheelVelocity", outputs.leftWheelVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/RightWheelVelocity", outputs.rightWheelVelocity.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/WheelSetpoint", outputs.wheelSetpoint.in(RotationsPerSecond));
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodAngle", outputs.hoodAngle.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodSetpoint", outputs.hoodSetpoint.in(Degrees));
		Logger.recordOutput(SUBSYSTEM_NAME + "/state", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/ReadyToShoot", readyToShoot());
		Logger.recordOutput(SUBSYSTEM_NAME + "/BallsPerSecond", numBallsShot / getStateTime()); //TODO: Remove later when testing is done
		Logger.recordOutput(SUBSYSTEM_NAME + "/NumBallsShot", numBallsShot);
		Logger.recordOutput(SUBSYSTEM_NAME + "/LeftWheelCurrent", outputs.leftWheelCurrent);
		Logger.recordOutput(SUBSYSTEM_NAME + "/RightWheelCurrent", outputs.rightWheelCurrent);
		Logger.recordOutput(SUBSYSTEM_NAME + "/HoodCurrent", outputs.hoodCurrent);
		Logger.recordOutput(SUBSYSTEM_NAME + "/FIlteredRPS", smoothFilter.lastValue());
	}

	public boolean readyToShoot() {
		return io.atHoodAngleSetpoint() && io.atWheelVelocitySetpoint();
	}

	@Override
	protected void stateExit() {
		cache = getState();
	}

	public List<TalonFX> getShooterMotors() {
		return io.getShooterMotors();
	}

	public TalonFX getHoodMotor() {
		return io.getHoodMotor();
	}
}
