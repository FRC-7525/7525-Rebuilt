package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import static frc.robot.Subsystems.Drive.TunerConstants.*;

import org.team7525.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates {
	FIELD_RELATIVE("Field Relative", () -> {
		double forward = -applyDeadband(DRIVER_CONTROLLER.getLeftY()) * kSpeedAt12Volts.in(MetersPerSecond);
		double strafe = -applyDeadband(DRIVER_CONTROLLER.getLeftX()) * kSpeedAt12Volts.in(MetersPerSecond);
		double rot = -applyDeadband(DRIVER_CONTROLLER.getRightX()) * ANGULAR_VELOCITY_LIMIT.in(RadiansPerSecond) * 0.1;
		Drive.getInstance().driveFieldRelative(forward, strafe, rot);
	}),
	ROBOT_RELATIVE("Robot Relative", () -> {
		double forward = applyDeadband(DRIVER_CONTROLLER.getLeftY()) * kSpeedAt12Volts.in(MetersPerSecond);
		double strafe = applyDeadband(DRIVER_CONTROLLER.getLeftX()) * kSpeedAt12Volts.in(MetersPerSecond);
		double rot = applyDeadband(DRIVER_CONTROLLER.getRightX()) * ANGULAR_VELOCITY_LIMIT.in(RadiansPerSecond);
		Drive.getInstance().driveRobotRelative(forward, strafe, rot);
	});

	private String stateString;
	private Runnable driveControl;

	DriveStates(String stateString, Runnable driveControl) {
		this.stateString = stateString;
		this.driveControl = driveControl;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public void driveRobot() {
		driveControl.run();
	}
}
