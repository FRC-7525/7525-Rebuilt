package frc.robot;

import static frc.robot.GlobalConstants.BLUE_ALLIANCE_BOUNDS;
import static frc.robot.GlobalConstants.RED_ALLIANCE_BOUNDS;
import static frc.robot.Subsystems.Manager.ManagerStates.IDLE;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Autonomous.AutoRoutines;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveStates;
import frc.robot.Subsystems.Manager.Manager;
import kotlin.Pair;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team7525.misc.CommandsUtil;
import org.team7525.misc.Tracer;

import choreo.auto.AutoChooser;

public class Robot extends LoggedRobot {

	private final AutoChooser autoChooser = new AutoChooser();
	private final AutoRoutines autoRoutines = new AutoRoutines();
	private final Manager manager = Manager.getInstance();
	
	public static boolean isRedAlliance = true;
	public static Pair<Translation2d, Translation2d> allianceZone = RED_ALLIANCE_BOUNDS;

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
		RobotController.setBrownoutVoltage(5.5);
		CommandScheduler.getInstance().unregisterAllSubsystems();
		System.gc();
		Drive.getInstance().zeroGyro();
		
		autoChooser.addRoutine("Right 2 Cycle", autoRoutines::Right2CycleRoutine);
		autoChooser.addRoutine("Left 2 Cycle", autoRoutines::Left2CycleRoutine);
		autoChooser.addRoutine("Right Sweeper 1 Cycle", autoRoutines::sweeperRight1Cycle);
		autoChooser.addRoutine("left Sweeper 1 Cycle", autoRoutines::sweeperLeft1Cycle);
		autoChooser.addRoutine("Right 1 Cycle Depot", autoRoutines::right1CycleDepot);
		SmartDashboard.putData("autoChooser", autoChooser);

		RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
	}
	
	@Override
	public void robotPeriodic() {
		Tracer.startTrace("RobotPeriodic");
		CommandScheduler.getInstance().run();
		Tracer.traceFunc("SubsystemManager", manager::periodic);
		Tracer.endTrace();
	}

	@Override
	public void autonomousInit() {
		Drive.getInstance().setState(DriveStates.AUTO);
	}

	@Override
	public void autonomousPeriodic() {

	}

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
		allianceZone = isRedAlliance ? RED_ALLIANCE_BOUNDS : BLUE_ALLIANCE_BOUNDS;
	}

	@Override
	public void disabledExit() {
		isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
	}
}
