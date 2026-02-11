package frc.robot.Subsytems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsytems.Shooter.ShooterConstants.*;

import frc.robot.GlobalConstants;
import frc.robot.Subsytems.Shooter.ShooterIO.ShooterIOOutputs;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem<ShooterStates> {

	private static Shooter instance;
	private final ShooterIO io;
	private ShooterIOOutputs outputs;
	private boolean shotTriggered = false;


	public static Shooter getInstance() {
		if (instance == null) {
			switch (GlobalConstants.ROBOT_MODE) {
				case REAL -> instance = new Shooter(new ShooterIOReal());
				case SIM -> instance = new Shooter(new ShooterIOSim());
				case TESTING -> instance = new Shooter(new ShooterIOTest());
			}
		}
		return instance;
	}

	private Shooter(ShooterIO io) {
		super(SUBSYSTEM_NAME, ShooterStates.OFF);
		this.io = io;
		outputs = new ShooterIOOutputs();

		SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Hood Angle", FIXED_SHOT_ANGLE.in(Degrees));
        SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Wheel Speed", FIXED_SHOT_SPEED.in(RotationsPerSecond));
        SmartDashboard.putBoolean(SUBSYSTEM_NAME + "/Shoot Now", false);
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

		Logger.recordOutput(SUBSYSTEM_NAME + "/Set Hood Angle", getState().getHoodAngle().in(Degrees));
        Logger.recordOutput(SUBSYSTEM_NAME + "/Set Wheel Speed", getState().getWheelVelocity().in(RotationsPerSecond));
	
	 boolean shootNow = SmartDashboard.getBoolean(SUBSYSTEM_NAME + "/Shoot Now", false);
        if (shootNow) {
            if (readyToShoot() && !shotTriggered) {
                io.fireOnce(); 
                shotTriggered = true;
                SmartDashboard.putBoolean(SUBSYSTEM_NAME + "/Shoot Now", false); // needed to not double shoot?
            }
        } else {
            shotTriggered = false;
        }
	}


	public boolean readyToShoot() {
		return io.atHoodAngleSetpoint() && io.atWheelVelocitySetpoint();
	}
}
