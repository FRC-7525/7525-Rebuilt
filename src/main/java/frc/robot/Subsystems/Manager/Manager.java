package frc.robot.Subsystems.Manager;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.GlobalConstants.Controllers.OPERATOR_CONTROLLER;
import static frc.robot.Subsystems.Manager.CurrentLimitConstants.*;
import static frc.robot.Subsystems.Manager.ManagerConstants.*;
import static frc.robot.Subsystems.Manager.ManagerStates.SCORING_AUTO;
import static frc.robot.Subsystems.Manager.ManagerStates.WINDING_TO_SCORE_AUTO;
import static frc.robot.Subsystems.Manager.ManagerStates.WINDING_UP;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveStates;
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

	private Timer shiftTimer = new Timer();
	private GameStates[] gameStates = ALLIANCE_WON_AUTONOMOUS;
	private GameStates currentGameState = GameStates.UNKNOWN;
	private int gameStateIndex = 0;
	private int numTimesYPressed = 0;

	private GameStates nextGameState = GameStates.TRANSITION_SHIFT;
	private double remainingPeriodTime = 10;

	private static final String USE_FMS = "FMS";
	private static final String FORCE_RED = "RED";
	private static final String FORCE_BLUE = "BLUE";
	private SendableChooser<String> autoWinnerChooser = new SendableChooser<>();

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
		addTrigger(ManagerStates.IDLE, ManagerStates.EXTENDED_IDLE, () -> DRIVER_CONTROLLER.getPOV() == 0);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.IDLE, () -> DRIVER_CONTROLLER.getPOV() == 180);

		//EXTENDED_IDLE <---> ZEROING
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.ZEROING, OPERATOR_CONTROLLER::getXButtonPressed);
		addTrigger(ManagerStates.ZEROING, ManagerStates.EXTENDED_IDLE, OPERATOR_CONTROLLER::getXButtonPressed);

		// IDLE/EXTENDED_IDLE --> INTAKING
		addTrigger(ManagerStates.IDLE, ManagerStates.INTAKING, DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.INTAKING, DRIVER_CONTROLLER::getXButtonPressed);

		// INTAKING --> EXTENDED_IDLE
		addTrigger(ManagerStates.INTAKING, ManagerStates.EXTENDED_IDLE, DRIVER_CONTROLLER::getXButtonPressed);

		addRunnableTrigger(
			() -> {
				numTimesYPressed++;

				if (numTimesYPressed == 3) {
					setState(WINDING_UP);
					numTimesYPressed = 1;
				}
			},
			() -> DRIVER_CONTROLLER.getYButtonPressed() || OPERATOR_CONTROLLER.getYButtonPressed()
		);

		// IDLE/EXTENDED_IDLE --> WINDING_UP
		addTrigger(ManagerStates.IDLE, ManagerStates.WINDING_UP, () -> numTimesYPressed == 1);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.WINDING_UP, () -> numTimesYPressed == 1);
		addTrigger(ManagerStates.IDLE, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ManagerStates.EXTENDED_IDLE, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);

		// INTAKING --> WINDING_UP
		addTrigger(ManagerStates.INTAKING, ManagerStates.WINDING_UP, () -> numTimesYPressed == 1);
		addTrigger(ManagerStates.INTAKING, ManagerStates.WINDING_UP, () -> numTimesYPressed == 1);
		addTrigger(ManagerStates.INTAKING, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);

		// WINDING_UP --> SHOOTING_HUB/SHOOTING_FIXED/SHOOTING_ALLIANCE
		addRunnableTrigger(() -> {
			numTimesYPressed = 2;
			setState(ManagerStates.SHOOTING_HUB);
		}, () -> isHubActive() && Math.abs(drive.getAngleDiffBetweenShooterAndTarget().in(Degrees)) < SHOOTER_TARGET_ANGLE_DIFF_DEGREES && Math.abs(drive.getVelocity().in(MetersPerSecond)) < CLOSE_TO_NOT_MOVING_MPS && getState() == WINDING_UP && drive.getState() == DriveStates.AIMLOCK_HUB && drive.isInTeamAllianceZone(drive.getPose()));
		addTrigger(ManagerStates.WINDING_UP, ManagerStates.SHOOTING_HUB, () -> numTimesYPressed == 2 && drive.isInTeamAllianceZone(drive.getPose()));
		addTrigger(ManagerStates.WINDING_UP_FIXED_SHOT, ManagerStates.SHOOTING_FIXED, DRIVER_CONTROLLER::getBButtonPressed);
		//TODO: fried state transition, kind of works but if you hold and release then it transitions (also doesn't work if you tap Y really fast)
		addTrigger(ManagerStates.WINDING_UP, ManagerStates.SHUTTLING, () -> numTimesYPressed == 2 && !drive.isInTeamAllianceZone(drive.getPose()));

		// SHOOTING --> WINDING_UP
		addTrigger(ManagerStates.SHOOTING_FIXED, ManagerStates.WINDING_UP_FIXED_SHOT, DRIVER_CONTROLLER::getBButtonPressed);

		// // IDLE <---> EXTENDING_CLIMBER
		// addTrigger(ManagerStates.IDLE, ManagerStates.EXTENDING_CLIMBER, OPERATOR_CONTROLLER::getRightBumperButtonPressed);

		// // EXTENDING_CLIMBER <---> RETRACTING_CLIMBER
		// addTrigger(ManagerStates.EXTENDING_CLIMBER, ManagerStates.RETRACTING_CLIMBER, OPERATOR_CONTROLLER::getLeftBumperButtonPressed);
		// addTrigger(ManagerStates.RETRACTING_CLIMBER, ManagerStates.EXTENDING_CLIMBER, OPERATOR_CONTROLLER::getLeftBumperButtonPressed);

		// Operator override HoodSnapDown
		addRunnableTrigger(shooter::toggleTrenchProtection, OPERATOR_CONTROLLER::getBButtonPressed);

		addRunnableTrigger(
			() -> {
				var redWon = autoWinnerChooser.getSelected().equalsIgnoreCase(FORCE_RED);
				if (redWon && Robot.isRedAlliance) gameStates = ALLIANCE_WON_AUTONOMOUS; // red won and we are red
				if (redWon && !Robot.isRedAlliance) gameStates = ALLIANCE_LOST_AUTONOMOUS; //red won and we are blue
				if (!redWon && Robot.isRedAlliance) gameStates = ALLIANCE_LOST_AUTONOMOUS; // red lost and we are red
				if (!redWon && !Robot.isRedAlliance) gameStates = ALLIANCE_WON_AUTONOMOUS; // red lost and we are blue
			},
			() -> !autoWinnerChooser.getSelected().equalsIgnoreCase(USE_FMS)
		);

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

		autoWinnerChooser.setDefaultOption("Use FMS", USE_FMS);
		autoWinnerChooser.addOption("Force Red Auto Win", FORCE_RED);
		autoWinnerChooser.addOption("Force Blue Auto Win", FORCE_BLUE);

		SmartDashboard.putData("Auto Winner Override", autoWinnerChooser);
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
			numTimesYPressed = 0;
		}

		double currentTime = shiftTimer.get();

		if (currentTime < 10) {
			currentGameState = gameStates[0];
			nextGameState = gameStates[1];
			remainingPeriodTime = 10 - currentTime;
		} else if (currentTime < 35) {
			currentGameState = gameStates[1];
			nextGameState = gameStates[2];
			remainingPeriodTime = 35 - currentTime;
		} else if (currentTime < 60) {
			currentGameState = gameStates[2];
			nextGameState = gameStates[3];
			remainingPeriodTime = 60 - currentTime;
		} else if (currentTime < 85) {
			currentGameState = gameStates[3];
			nextGameState = gameStates[4];
			remainingPeriodTime = 85 - currentTime;
		} else if (currentTime < 110) {
			currentGameState = gameStates[4];
			nextGameState = gameStates[5];
			remainingPeriodTime = 110 - currentTime;
		} else if (currentTime < 140) {
			currentGameState = gameStates[5];
			remainingPeriodTime = 140 - currentTime;
		}

		Logger.recordOutput("Manager/TIME UNTIL NEXT SHIFT", remainingPeriodTime);
		Logger.recordOutput("Manager/CURRENT HUB STATE", currentGameState.getStateString());
		Logger.recordOutput("Manager/NEXT HUB STATE", nextGameState.getStateString());
	}

	public GameStates getCurrentGameState() {
		return currentGameState;
	}

	public boolean isHubActive() {
		return !(currentGameState == GameStates.HUB_NOT_ACTIVE);
	}

	public void initalizeShiftTimer() {
		String gameData = DriverStation.getGameSpecificMessage();
		boolean redWon = true;
		if (gameData.length() > 0) {
			if (gameData.charAt(0) == 'R') redWon = true;
			else if (gameData.charAt(0) == 'B') redWon = false;
			else {
				gameStates = UNKNOWN_ALLIANCE_WON;
				return;
			}
		}

		if (redWon && Robot.isRedAlliance) gameStates = ALLIANCE_WON_AUTONOMOUS; // red won and we are red
		if (redWon && !Robot.isRedAlliance) gameStates = ALLIANCE_LOST_AUTONOMOUS; //red won and we are blue
		if (!redWon && Robot.isRedAlliance) gameStates = ALLIANCE_LOST_AUTONOMOUS; // red lost and we are red
		if (!redWon && !Robot.isRedAlliance) gameStates = ALLIANCE_WON_AUTONOMOUS; // red lost and we are blue

		shiftTimer.start();
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
