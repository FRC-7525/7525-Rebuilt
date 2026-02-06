package frc.robot;

import static frc.robot.Subsystems.Manager.ManagerStates.IDLE;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.AngleAndShootCommand;
import frc.robot.Commands.AutoCommands;
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
	private SendableChooser<Command> autoChooser;
	private AutoCommands autoCommands = new AutoCommands(this);

	public static boolean isRedAlliance = true;

	@Override
	public void robotInit() {
		autoChooser = AutoBuilder.buildAutoChooser();

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
		SmartDashboard.putData("Auto Chooser", autoChooser);

		NamedCommands.registerCommand("Intake", autoCommands.intake());
		NamedCommands.registerCommand("Return to Idle", autoCommands.returnToIdle());
		NamedCommands.registerCommand("Start Winding Up", autoCommands.startWindingUp());
		NamedCommands.registerCommand("Wind and Intake", autoCommands.windAndIntake());

		NamedCommands.registerCommand("Shooting Hub", new AngleAndShootCommand(this));
	}

	public Manager getManager() {
		return manager;
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
		Command autoCommand = autoChooser.getSelected();
		if (autoCommand != null) {
			autoCommand.schedule();
		}
	}

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
	}

	@Override
	public void disabledExit() {
		isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
	}
}
