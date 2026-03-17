package frc.robot.Subsystems.Manager;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.GlobalConstants.Controllers.OPERATOR_CONTROLLER;
import static frc.robot.Subsystems.Manager.CurrentLimitConstants.*;
import static frc.robot.Subsystems.Manager.ManagerConstants.*;
import static frc.robot.Subsystems.Manager.ManagerStates.SCORING_AUTO;
import static frc.robot.Subsystems.Manager.ManagerStates.WINDING_TO_SCORE_AUTO;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants;
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
		vision = Vision.getInstance();

		// IDLE <---> EXTENDED_IDLE
		addTrigger(ManagerStates.IDLE, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getRightBumperButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.IDLE, DRIVER_CONTROLLER::getRightBumperButtonPressed);

		//EXTENDED_IDLE <---> ZEROING
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.ZEROING, OPERATOR_CONTROLLER::getXButtonPressed);
		addTrigger(ManagerStates.ZEROING, ManagerStates.EXTENDED_IDLE, OPERATOR_CONTROLLER::getXButtonPressed);

		// IDLE/EXTENDED_IDLE --> INTAKING
		addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING, DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.INTAKING, DRIVER_CONTROLLER::getXButtonPressed);

		// INTAKING --> EXTENDED_IDLE
		addTrigger(ManagerStates.INTAKING, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getXButtonPressed);

		// IDLE/EXTENDED_IDLE --> WINDING_UP
		addTrigger(ManagerStates.IDLE, ManagerStates.WINDING_UP, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.WINDING_UP, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.WINDING_UP, OPERATOR_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.IDLE, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);

		// INTAKING --> WINDING_UP
		addTrigger(ManagerStates.INTAKING, ManagerStates.WINDING_UP, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.INTAKING, ManagerStates.WINDING_UP, OPERATOR_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.INTAKING, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);

		// WINDING_UP --> SHOOTING_HUB/SHOOTING_FIXED/SHOOTING_ALLIANCE
		addTrigger(ManagerStates.WINDING_UP, ManagerStates.SHOOTING_HUB, () -> DRIVER_CONTROLLER.getYButtonPressed() && drive.isInTeamAllianceZone(drive.getPose()));
		addTrigger(ManagerStates.WINDING_UP_FIXED_SHOT, ManagerStates.SHOOTING_FIXED, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ManagerStates.WINDING_UP, ManagerStates.SHUTTLING, () -> DRIVER_CONTROLLER.getYButtonPressed() && !drive.isInTeamAllianceZone(drive.getPose())); // If we're not in the alliance zone, we should be shooting alliance shots not hub shots so shuttle instead of shoot directly from winding up

		// SHOOTING --> WINDING_UP
		//Done to simplify control and reduce wind-up time when needing to move in between shots (like if the hub is in the way while shuttling)
		addTrigger(ManagerStates.SHOOTING_HUB, ManagerStates.WINDING_UP, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.SHUTTLING, ManagerStates.WINDING_UP, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.SHOOTING_FIXED, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);

		// // IDLE <---> EXTENDING_CLIMBER
		// addTrigger(ManagerStates.IDLE, ManagerStates.EXTENDING_CLIMBER, OPERATOR_CONTROLLER::getRightBumperButtonPressed);

		// // EXTENDING_CLIMBER <---> RETRACTING_CLIMBER
		// addTrigger(ManagerStates.EXTENDING_CLIMBER, ManagerStates.RETRACTING_CLIMBER, OPERATOR_CONTROLLER::getLeftBumperButtonPressed);
		// addTrigger(ManagerStates.RETRACTING_CLIMBER, ManagerStates.EXTENDING_CLIMBER, OPERATOR_CONTROLLER::getLeftBumperButtonPressed);

		// Operator override HoodSnapDown
		addRunnableTrigger(shooter::toggleTrenchProtection, OPERATOR_CONTROLLER::getBButtonPressed);

		// ----------------------------------------------  AUTO EXCLUSIVE TRIGGERS  -------------------------------------------------------

		// addTrigger(ManagerStates.WINDING_TO_SCORE_AUTO, ManagerStates.SCORING_AUTO, () -> Drive.getInstance().isInTeamAllianceZone(Drive.getInstance().getPose()) && Math.abs(Drive.getInstance().getAngleDiffBetweenShooterAndTarget().in(Degrees)) > AutoAlignConstants.MAX_YAW_ERROR.in(Degrees));
		addTrigger(SCORING_AUTO, WINDING_TO_SCORE_AUTO, () -> !Drive.getInstance().isInTeamAllianceZone(Drive.getInstance().getPose()) || !(Math.abs(Drive.getInstance().getAngleDiffBetweenShooterAndTarget().in(Degrees)) > AutoAlignConstants.MAX_YAW_ERROR.in(Degrees)));

		// -----------------------------------------------  CURRENT LIMITS SETUP  -------------------------------------------------------
		drive.getDriveMotors().forEach(motor -> motor.getConfigurator().apply(getState().getCurrentLimiterState().getDriveLimit()));
		drive.getTurnMotors().forEach(motor -> motor.getConfigurator().apply(getState().getCurrentLimiterState().getTurnLimit()));
		shooter.getShooterMotors().forEach(motor -> motor.getConfigurator().apply(getState().getCurrentLimiterState().getShooterLimit()));
		shooter.getHoodMotor().getConfigurator().apply(HOOD_LIMITS);
		hopper.getSpinMotor().getConfigurator().apply(SPINDEXER_LIMITS);
		intake.getPivotMotor().getConfigurator().apply(INTAKE_PIVOT_LIMITS);
		intake.getSpinMotor().getConfigurator().apply(INTAKE_WHEEL_LIMITS);
		hopper.getKickerMotor1().getConfigurator().apply(KICKER_LIMITS);
		hopper.getKickerMotor2().getConfigurator().apply(KICKER_LIMITS_2);
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
		Logger.recordOutput(SUBSYSTEM_NAME + "/STATE TIME", getStateTime());
		Logger.recordOutput(SUBSYSTEM_NAME + "/HUB ACTIVE", isHubActive());

		// Set subsystem states
		shooter.setState(getState().getShooterState());
		hopper.setState(getState().getHopperState());
		intake.setState(getState().getIntakeState());

		Tracer.traceFunc("ShooterPeriodic", shooter::periodic);
		Tracer.traceFunc("HopperPeriodic", hopper::periodic);
		Tracer.traceFunc("IntakePeriodic", intake::periodic);
		Tracer.traceFunc("DrivePeriodic", drive::periodic);
		Tracer.traceFunc("VisionPeriodic", vision::periodic);

		// Emergency stop to IDLE
		if (DRIVER_CONTROLLER.getStartButton() || OPERATOR_CONTROLLER.getStartButton()) {
			setState(ManagerStates.EXTENDED_IDLE);
		}
	}

	public boolean isHubActive() {
		return true; // TODO: implement this
	}

	@Override
	protected void stateInit() {
		// Update current limits for drive, turn, and shooter based on the current limiter state of the new manager state
		drive.getDriveMotors().forEach(motor -> motor.getConfigurator().apply(getState().getCurrentLimiterState().getDriveLimit()));
		drive.getTurnMotors().forEach(motor -> motor.getConfigurator().apply(getState().getCurrentLimiterState().getTurnLimit()));
		shooter.getShooterMotors().forEach(motor -> motor.getConfigurator().apply(getState().getCurrentLimiterState().getShooterLimit()));
	}

	@Override
	public void periodic() {
		super.periodic();
		WiltingRoseControllerUtility.resetWiltingRoseControllers();
	}
}
