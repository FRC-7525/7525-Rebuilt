package frc.robot;

import static frc.robot.Subsystems.Manager.ManagerStates.IDLE;
import static frc.robot.Subsystems.Shooter.ShooterConstants.BLUE_HUB_BR;
import static frc.robot.Subsystems.Shooter.ShooterConstants.BLUE_HUB_TL;
import static frc.robot.Subsystems.Shooter.ShooterConstants.HUB_POSES;
import static frc.robot.Subsystems.Shooter.ShooterConstants.LEFT_DEEP_POSES;
import static frc.robot.Subsystems.Shooter.ShooterConstants.LEFT_SHALLOW_POSES;
import static frc.robot.Subsystems.Shooter.ShooterConstants.RED_HUB_BR;
import static frc.robot.Subsystems.Shooter.ShooterConstants.RED_HUB_TL;
import static frc.robot.Subsystems.Shooter.ShooterConstants.RIGHT_DEEP_POSES;
import static frc.robot.Subsystems.Shooter.ShooterConstants.RIGHT_SHALLOW_POSES;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Manager.Manager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team7525.misc.CommandsUtil;
import org.team7525.misc.Tracer;

public class Robot extends LoggedRobot {

	private final Manager manager = Manager.getInstance();

	public static boolean isRedAlliance = true;
	public static Translation2d hubRegion_TL = RED_HUB_TL;
	public static Translation2d hubRegion_BR = RED_HUB_BR;
	public static Pose2d hubPose = HUB_POSES.getRedPose();
	public static Pose2d leftDeep = LEFT_DEEP_POSES.getRedPose();
	public static Pose2d leftShallow = LEFT_SHALLOW_POSES.getRedPose();
	public static Pose2d rightDeep = LEFT_DEEP_POSES.getRedPose();
	public static Pose2d rightShallow = LEFT_DEEP_POSES.getRedPose();

	@Override
	public void robotInit() {
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL:
				Logger.addDataReceiver(new NT4Publisher());
				Logger.addDataReceiver(new WPILOGWriter());
				break;
			case SIM:
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case TESTING:
				Logger.addDataReceiver(new NT4Publisher());
				break;
		}

		// Lots and lots of trolling
		Logger.start();
		CommandsUtil.logCommands();
		DriverStation.silenceJoystickConnectionWarning(true);
		CommandScheduler.getInstance().unregisterAllSubsystems();
		System.gc();
		Drive.getInstance().zeroGyro();
	}

	@Override
	public void robotPeriodic() {
		Tracer.startTrace("RobotPeriodic");
		CommandScheduler.getInstance().run();
		Tracer.traceFunc("SubsystemManager", manager::periodic);
		Tracer.endTrace();
	}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopInit() {
		manager.setState(IDLE);
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void disabledInit() {
		System.gc();
	}

	@Override
	public void disabledPeriodic() {
		isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
		hubPose = isRedAlliance ? HUB_POSES.getRedPose() : HUB_POSES.getBluePose();
		leftDeep = isRedAlliance ? LEFT_DEEP_POSES.getRedPose() : LEFT_DEEP_POSES.getBluePose();
		leftShallow = isRedAlliance ? LEFT_SHALLOW_POSES.getRedPose() : LEFT_SHALLOW_POSES.getBluePose();
		rightDeep = isRedAlliance ? RIGHT_DEEP_POSES.getRedPose() : RIGHT_DEEP_POSES.getBluePose();
		rightShallow = isRedAlliance ? RIGHT_SHALLOW_POSES.getRedPose() : RIGHT_SHALLOW_POSES.getBluePose();
		hubRegion_TL = isRedAlliance ? RED_HUB_TL : BLUE_HUB_TL;
		hubRegion_BR = isRedAlliance ? RED_HUB_BR : BLUE_HUB_BR;
	}

	@Override
	public void disabledExit() {
		isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
		hubPose = isRedAlliance ? HUB_POSES.getRedPose() : HUB_POSES.getBluePose();
		leftDeep = isRedAlliance ? LEFT_DEEP_POSES.getRedPose() : LEFT_DEEP_POSES.getBluePose();
		leftShallow = isRedAlliance ? LEFT_SHALLOW_POSES.getRedPose() : LEFT_SHALLOW_POSES.getBluePose();
		rightDeep = isRedAlliance ? RIGHT_DEEP_POSES.getRedPose() : RIGHT_DEEP_POSES.getBluePose();
		rightShallow = isRedAlliance ? RIGHT_SHALLOW_POSES.getRedPose() : RIGHT_SHALLOW_POSES.getBluePose();
		hubRegion_TL = isRedAlliance ? RED_HUB_TL : BLUE_HUB_TL;
		hubRegion_BR = isRedAlliance ? RED_HUB_BR : BLUE_HUB_BR;
	}
}
