package frc.robot;

import static frc.robot.Subsystems.Manager.ManagerStates.IDLE;

import edu.wpi.first.wpilibj.DriverStation;
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
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}
}
