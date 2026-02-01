package frc.robot.Subsystems.Manager;

import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.GlobalConstants.Controllers.OPERATOR_CONTROLLER;
import static frc.robot.Subsystems.Manager.ManagerConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Hopper.Hopper;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Vision.Vision;
import org.littletonrobotics.junction.Logger;
import org.team7525.misc.Tracer;
import org.team7525.subsystem.Subsystem;

public class Manager extends Subsystem<ManagerStates> {

	private static Manager instance;
	private Drive drive;
	private Shooter shooter;
	private Hopper hopper;
	private Intake intake;
	private Climber climber;
	private Vision vision;

	public static Manager getInstance() {
		if (instance == null) {
			new Manager();
		}
		return instance;
	}

	private Manager() {
		super(SUBSYSTEM_NAME, ManagerStates.IDLE);
		instance = this;
		drive = Drive.getInstance();
		shooter = Shooter.getInstance();
		hopper = Hopper.getInstance();
		intake = Intake.getInstance();
		climber = Climber.getInstance();
		vision = Vision.getInstance();

		// IDLE <---> EXTENDED_IDLE
		addTrigger(ManagerStates.IDLE, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getRightBumperButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.IDLE, DRIVER_CONTROLLER::getRightBumperButtonPressed);

		// IDLE/EXTENDED_IDLE --> INTAKING
		addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING, DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.INTAKING, DRIVER_CONTROLLER::getXButtonPressed);

		// INTAKING --> EXTENDED_IDLE
		addTrigger(ManagerStates.INTAKING, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getXButtonPressed);

		// IDLE/EXTENDED_IDLE --> WINDING_UP
		addTrigger(ManagerStates.IDLE, ManagerStates.WINDING_UP, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.WINDING_UP, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.IDLE, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);

		// INTAKING --> WINDING_UP
		addTrigger(ManagerStates.INTAKING, ManagerStates.WINDING_UP, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.INTAKING, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);

		// WINDING_UP --> SHOOTING_HUB/SHOOTING_FIXED/SHOOTING_ALLIANCE
		addTrigger(ManagerStates.WINDING_UP, ManagerStates.SHOOTING_HUB, () -> shooter.readyToShoot() && drive.isAtAllianceShootingPosition());
		addTrigger(ManagerStates.WINDING_UP_FIXED_SHOT, ManagerStates.SHOOTING_FIXED, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ManagerStates.WINDING_UP, ManagerStates.SHUTTLING, () -> shooter.readyToShoot() && !drive.isAtAllianceShootingPosition());

		// SHOOTING --> EXTENDED_IDLE
		addTrigger(ManagerStates.SHOOTING_HUB, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.SHUTTLING, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.SHOOTING_FIXED, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getBButtonPressed);

		// IDLE <---> EXTENDING_CLIMBER
		addTrigger(ManagerStates.IDLE, ManagerStates.EXTENDING_CLIMBER, OPERATOR_CONTROLLER::getRightBumperButtonPressed);

		// EXTENDING_CLIMBER <---> RETRACTING_CLIMBER
		addTrigger(ManagerStates.EXTENDING_CLIMBER, ManagerStates.RETRACTING_CLIMBER, OPERATOR_CONTROLLER::getRightBumperButtonPressed);
		addTrigger(ManagerStates.RETRACTING_CLIMBER, ManagerStates.EXTENDING_CLIMBER, OPERATOR_CONTROLLER::getLeftBumperButtonPressed);
		addTrigger(ManagerStates.EXTENDING_CLIMBER, ManagerStates.IDLE, OPERATOR_CONTROLLER::getLeftBumperButtonPressed);
	}

	@Override
	public void runState() {
		var allianceOpt = DriverStation.getAlliance();
		if (allianceOpt.isEmpty()) {
			// Alliance not yet known (early init); record as UNKNOWN instead of calling get().
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "UNKNOWN");
		} else if (allianceOpt.get() == DriverStation.Alliance.Red) {
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "RED");
		} else {
			Logger.recordOutput(SUBSYSTEM_NAME + "/ALLIANCE COLOR", "BLUE");
		}

		Logger.recordOutput(SUBSYSTEM_NAME + "/STATE", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/InAllianceShootingPosition", drive.isAtAllianceShootingPosition());
		Logger.recordOutput(SUBSYSTEM_NAME + "/STATE TIME", getStateTime());
		Logger.recordOutput(SUBSYSTEM_NAME + "/HUB ACTIVE", isHubActive());

		// Set subsystem states
		shooter.setState(getState().getShooterState());
		hopper.setState(getState().getHopperState());
		intake.setState(getState().getIntakeState());
		climber.setState(getState().getClimberState());

		Tracer.traceFunc("ShooterPeriodic", shooter::periodic); // SHould these be used with Tracer? idk what that does fr
		Tracer.traceFunc("HopperPeriodic", hopper::periodic);
		Tracer.traceFunc("IntakePeriodic", intake::periodic);
		Tracer.traceFunc("ClimberPeriodic", climber::periodic);
		Tracer.traceFunc("DrivePeriodic", drive::periodic);
		Tracer.traceFunc("VisionPeriodic", vision::periodic);
		// Emergency stop to IDLE
		if (DRIVER_CONTROLLER.getStartButtonPressed()) {
			setState(ManagerStates.IDLE);
		}
	}

	public boolean isHubActive() {
		return true; // TODO: implement this idk how
	}

	@Override
	public void periodic() {
		super.periodic();
		WiltingRoseControllerUtils.resetWiltingRoseControllers();
	}
}
