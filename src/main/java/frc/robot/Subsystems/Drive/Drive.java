package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.GlobalConstants.Controllers.DEADBAND;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.GlobalConstants.Controllers.OPERATOR_CONTROLLER;
import static frc.robot.GlobalConstants.FIELD;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants.*;
import static frc.robot.Subsystems.Drive.DriveConstants.ANGULAR_VELOCITY_LIMIT;
import static frc.robot.Subsystems.Drive.DriveConstants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION;
import static frc.robot.Subsystems.Drive.DriveConstants.RED_ALLIANCE_PERSPECTIVE_ROTATION;
import static frc.robot.Subsystems.Drive.DriveConstants.SUBSYSTEM_NAME;
import static frc.robot.Subsystems.Drive.TunerConstants.kSpeedAt12Volts;
import static frc.robot.Subsystems.Shooter.ShooterConstants.BLUE_HUB_POSE;
import static frc.robot.Subsystems.Shooter.ShooterConstants.RED_HUB_POSE;
import static frc.robot.Subsystems.Shooter.ShooterConstants.ROBOT_TO_SHOOTER;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants.Obstacles;
import frc.robot.Subsystems.Drive.AutoAlign.MathHelpers;
import frc.robot.Subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import java.util.List;
import kotlin.Pair;
import org.littletonrobotics.junction.Logger;
import org.team7525.autoAlign.RepulsorFieldPlanner;
import org.team7525.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {

	private static Drive instance;

	private DriveIO driveIO;

	private boolean isFieldRelative;
	private Pair<Translation2d, Translation2d> allianceZone;
	private boolean robotMirrored = false;
	private Pose2d lastPose = new Pose2d();
	private Pose2d targetPose = Pose2d.kZero;
	private Pose2d sotmTarget = Pose2d.kZero;
	private double lastTime = 0;

	private boolean usedRepulsor = false;
	private final RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner(Obstacles.FIELD_OBSTACLES, Obstacles.WALLS, (ROBOT_MODE == RobotMode.SIM));
	private final Field2d field = new Field2d();

	private final ProfiledPIDController rotationController;
	private final ProfiledPIDController translationalController;
	private final PIDController shooterYawController;
	private final PIDController repulsorTranslationController;
	private final PIDController repulsorRotationalController;
	private final PIDController shooterYawControllerFast;

	private double driveErrorAbs;
	private double thetaErrorAbs;
	private double ffMinRadius = 0.2, ffMaxRadius = 1.0;

	/**
	 * Constructs a new Drive subsystem with the given DriveIO.
	 *
	 * @param driveIO The DriveIO object used for controlling the drive system.
	 */
	private Drive() {
		super("Drive", DriveStates.NORMAL);
		this.driveIO = switch (ROBOT_MODE) {
			case REAL -> new DriveIOReal();
			case SIM -> new DriveIOSim();
			case TESTING -> new DriveIOReal();
		};

		this.shooterYawController = SHOOTER_YAW_CONTROLLER.get();
		this.shooterYawControllerFast = SHOOTER_YAW_CONTROLLER_FAST.get();
		this.rotationController = SCALED_FF_ROTATIONAL_CONTROLLER.get();
		this.translationalController = SCALED_FF_TRANSLATIONAL_CONTROLLER.get();

		this.repulsorTranslationController = REPULSOR_TRANSLATIONAL_CONTROLLER.get();
		this.repulsorRotationalController = REPULSOR_ROTATIONAL_CONTROLLER.get();

		this.shooterYawController.setTolerance(ANGLE_ERROR_MARGIN.in(Radians));
		this.repulsorTranslationController.setTolerance(DISTANCE_ERROR_MARGIN.in(Meters));
		this.translationalController.setTolerance(DISTANCE_ERROR_MARGIN.in(Meters));
		this.repulsorRotationalController.setTolerance(ANGLE_ERROR_MARGIN.in(Radians));
		this.rotationController.setTolerance(ANGLE_ERROR_MARGIN.in(Radians));

		this.shooterYawController.enableContinuousInput(MIN_HEADING_ANGLE.in(Radians), MAX_HEADING_ANGLE.in(Radians));
		this.rotationController.enableContinuousInput(MIN_HEADING_ANGLE.in(Radians), MAX_HEADING_ANGLE.in(Radians));
		this.repulsorRotationalController.enableContinuousInput(MIN_HEADING_ANGLE.in(Radians), MAX_HEADING_ANGLE.in(Radians));
		this.shooterYawControllerFast.enableContinuousInput(MIN_HEADING_ANGLE.in(Radians), MAX_HEADING_ANGLE.in(Radians));
		this.isFieldRelative = true;

		// Zero Gyro
		addRunnableTrigger(
			() -> {
				driveIO.zeroGyro();
			},
			OPERATOR_CONTROLLER::getBackButtonPressed
		);
		addRunnableTrigger(
			() -> {
				driveIO.zeroGyro();
			},
			DRIVER_CONTROLLER::getBackButtonPressed
		);
		// addRunnableTrigger(() -> isFieldRelative = !isFieldRelative, DRIVER_CONTROLLER::getBackButtonPressed);
		addTrigger(DriveStates.NORMAL, DriveStates.AIMLOCK_HUB, DRIVER_CONTROLLER::getLeftBumperButtonPressed);
		addTrigger(DriveStates.AIMLOCK_HUB, DriveStates.NORMAL, DRIVER_CONTROLLER::getLeftBumperButtonPressed);
	}

	/**
	 * Returns the singleton instance of the Drive subsystem.
	 *
	 * @return The Drive Instance.
	 */
	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	@Override
	public void runState() {
		sotmTarget = Robot.isRedAlliance ? RED_HUB_POSE : BLUE_HUB_POSE;
		SmartDashboard.putData("Sus Fast COntroller", shooterYawControllerFast);
		SmartDashboard.putData("Shooter CONTROLLER", shooterYawController);
		if (DriverStation.isDisabled()) robotMirrored = false;

		// Zero on init/when first disabled
		if (!robotMirrored && !DriverStation.isDisabled()) {
			DriverStation.getAlliance()
				.ifPresent(allianceColor -> {
					driveIO.getDrive().setOperatorPerspectiveForward(allianceColor == Alliance.Red ? RED_ALLIANCE_PERSPECTIVE_ROTATION : BLUE_ALLIANCE_PERSPECTIVE_ROTATION);
					robotMirrored = true;
				});
		}
		logOutputs(driveIO.getDrive().getState());

		switch (getState()) {
			case NORMAL:
				executeDriveInstruction(DRIVER_CONTROLLER.getLeftY() * kSpeedAt12Volts.in(MetersPerSecond), DRIVER_CONTROLLER.getLeftX() * kSpeedAt12Volts.in(MetersPerSecond), -DRIVER_CONTROLLER.getRightX() * ANGULAR_VELOCITY_LIMIT.in(RadiansPerSecond) * 0.1, isFieldRelative);
				break;
			case AIMLOCK_ALLIANCE_LEFT_SHALLOW:
			case AIMLOCK_ALLIANCE_LEFT_DEEP:
			case AIMLOCK_ALLIANCE_RIGHT_DEEP:
			case AIMLOCK_ALLIANCE_RIGHT_SHALLOW:
			case AIMLOCK_HUB:
				Pose2d target = sotmTarget;
				Pose2d shooterPosition = getPose().plus(new Transform2d(ROBOT_TO_SHOOTER.getTranslation().toTranslation2d(), ROBOT_TO_SHOOTER.getRotation().toRotation2d()));
				Pose2d shooterToTarget = target.relativeTo(shooterPosition);
				double turnValue;
				if (Math.abs(shooterToTarget.getTranslation().getAngle().getDegrees()) >= SWITCH_DIST.in(Degrees)) {
					turnValue = shooterYawControllerFast.calculate(shooterToTarget.getTranslation().getAngle().getRadians(), Math.PI);
				} else {
					turnValue = Math.abs(shooterToTarget.getTranslation().getAngle().getDegrees()) > MAX_YAW_ERROR.in(Degrees) ? shooterYawController.calculate(shooterToTarget.getTranslation().getAngle().getRadians(), Math.PI) : 0;
				}
				executeAutoAlignDriveInstruction(DRIVER_CONTROLLER.getLeftY() * kSpeedAt12Volts.in(MetersPerSecond), DRIVER_CONTROLLER.getLeftX() * kSpeedAt12Volts.in(MetersPerSecond), turnValue, true);
				Logger.recordOutput("shooter/target", target);
				Logger.recordOutput("shooter/Angle Diff To Target", shooterToTarget.getTranslation().getAngle().getDegrees());
				Logger.recordOutput("shooter/ShooterPosition", shooterPosition);
				break;
			case AA_NEUTRAL:
			case AA_TOWER_LEFT:
			case AA_TOWER_RIGHT:
				targetPose = Robot.isRedAlliance ? getState().getTargetPosePair().getRedPose() : getState().getTargetPosePair().getBluePose();
				if (!isInTeamAllianceZone(getPose()) || !isInTeamAllianceZone(targetPose)) {
					executeRepulsorAutoAlign();
					usedRepulsor = true;
				} else {
					if (usedRepulsor) {
						resetPID();
						usedRepulsor = false;
					}
					executeScaledFeedforwardAutoAlign();
				}
				break;
		}
		field.setRobotPose(getPose());
		SmartDashboard.putData("Field", field);
	}

	/**
	 * Logs the outputs of the drive system.
	 *
	 * @param state The current state of the SwerveDrive.
	 */

	public void executeDriveInstruction(double xVelocity, double yVelocity, double angularVelocity, boolean fieldRelative) {
		if (fieldRelative) {
			driveIO.setControl(
				new SwerveRequest.FieldCentric()
					.withDeadband(DEADBAND)
					.withVelocityX(xVelocity)
					.withVelocityY(yVelocity)
					.withRotationalRate(angularVelocity)
					.withRotationalRate(angularVelocity)
					.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
					.withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
					.withRotationalDeadband(1)
			);
		} else {
			driveIO.setControl(
				new SwerveRequest.RobotCentric()
					.withDeadband(DEADBAND)
					.withVelocityX(xVelocity)
					.withVelocityY(yVelocity)
					.withRotationalRate(angularVelocity)
					.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
					.withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
					.withRotationalDeadband(1)
			);
		}
	}

	public void executeAutoAlignDriveInstruction(double xVelocity, double yVelocity, double angularVelocity, boolean hasDriverControl) {
		if (hasDriverControl) {
			driveIO.setControl(
				new SwerveRequest.RobotCentric().withDeadband(DEADBAND).withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
			);
		} else {
			driveIO.setControl(new SwerveRequest.RobotCentric().withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
		}
	}

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
		if (ROBOT_MODE == RobotMode.REAL) {
			driveIO.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestamp), visionMeasurementStdDevs);
		} else {
			driveIO.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		}
	}

	public void executeRepulsorAutoAlign() {
		Pose2d currentPose = getPose();
		// Set repulsor goal and get command
		repulsor.setGoal(targetPose.getTranslation());
		SwerveSample sample = repulsor.getCmd(currentPose, getRobotRelativeSpeeds(), MAX_SPEED.in(MetersPerSecond), USE_GOAL, targetPose.getRotation());

		// Extract and modify chassis speeds with additional control
		var targetSpeeds = sample.getChassisSpeeds();
		targetSpeeds.vxMetersPerSecond += repulsorTranslationController.calculate(currentPose.getX(), sample.x);
		targetSpeeds.vyMetersPerSecond += repulsorTranslationController.calculate(currentPose.getY(), sample.y);
		targetSpeeds.omegaRadiansPerSecond += repulsorRotationalController.calculate(currentPose.getRotation().getRadians(), sample.heading);

		Logger.recordOutput("AutoAlign/TranslationRepulsor X", targetSpeeds.vxMetersPerSecond);
		Logger.recordOutput("AutoAlign/TranslationRepulsor Y", targetSpeeds.vyMetersPerSecond);
		// No more race conditions :Sob:
		if (Robot.isRedAlliance) {
			executeDriveInstruction(-targetSpeeds.vxMetersPerSecond, -targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, true);
		} else {
			executeDriveInstruction(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond, true);
		}
	}

	// 254's implementation of simple PID to pose auto-align
	public void executeScaledFeedforwardAutoAlign() {
		Logger.recordOutput("AutoAlign/ProfiledVelSetpoint", translationalController.getSetpoint().velocity);
		Logger.recordOutput("AutoAlign/ProfiledPosSetpoint", translationalController.getSetpoint().position);
		Pose2d currentPose = getPose();
		// Apply scalar drive with feedforward
		double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
		double ffScaler = MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

		driveErrorAbs = currentDistance;
		translationalController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()), translationalController.getSetpoint().velocity);

		// Calculate translation velocity scalar with PID and FF scaling
		double translationVelocityScalar = (translationalController.getSetpoint().velocity * ffScaler) + translationalController.calculate(driveErrorAbs, 0.0);
		if (currentDistance < translationalController.getPositionTolerance()) {
			translationVelocityScalar = 0;
		}

		// Calculate rotation velocity with PID and FF scaling
		double thetaVelocity = rotationController.getSetpoint().velocity * ffScaler + rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

		thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
		if (thetaErrorAbs < rotationController.getPositionTolerance()) {
			thetaVelocity = 0;
		}

		// Calculate final translation velocity
		var translationVelocity = MathHelpers.pose2dFromRotation(currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()).transformBy(MathHelpers.transform2dFromTranslation(new Translation2d(translationVelocityScalar, 0.0))).getTranslation();

		// Apply drive commands with alliance compensation
		if (Robot.isRedAlliance) {
			executeAutoAlignDriveInstruction(-translationVelocity.getX(), translationVelocity.getY(), thetaVelocity, false);
		} else {
			executeAutoAlignDriveInstruction(translationVelocity.getX(), translationVelocity.getY(), thetaVelocity, false);
		}
	}

	boolean isInTeamAllianceZone(Pose2d currentPose) {
		double x = currentPose.getX();
		double y = currentPose.getY();
		if (!(x > allianceZone.getFirst().getX() && x < allianceZone.getSecond().getX())) return false;
		if (!(y > allianceZone.getFirst().getY() && y < allianceZone.getSecond().getY())) return false;
		return true;
	}

	public void zeroGyro() {
		driveIO.zeroGyro();
	}

	// Util
	public Pose2d getPose() {
		return driveIO.getDrive().getState().Pose;
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return driveIO.getDrive().getState().Speeds;
	}

	public LinearVelocity getVelocity() {
		return MetersPerSecond.of(Math.hypot(driveIO.getDrive().getState().Speeds.vxMetersPerSecond, driveIO.getDrive().getState().Speeds.vyMetersPerSecond));
	}

	public TunerSwerveDrivetrain getDriveTrain() {
		return driveIO.getDrive();
	}

	public void logOutputs(SwerveDriveState state) {
		Logger.recordOutput(SUBSYSTEM_NAME + "/Robot Pose", state.Pose);
		Logger.recordOutput(SUBSYSTEM_NAME + "/Current Time", Utils.getSystemTimeSeconds());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Chassis Speeds", state.Speeds);
		Logger.recordOutput(SUBSYSTEM_NAME + "/velocity", Units.metersToFeet(Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond)));
		Logger.recordOutput(SUBSYSTEM_NAME + "/swerveModuleStates", state.ModuleStates);
		Logger.recordOutput(SUBSYSTEM_NAME + "/swerveModulePosition", state.ModulePositions);
		Logger.recordOutput(SUBSYSTEM_NAME + "/Translation Difference", state.Pose.getTranslation().minus(lastPose.getTranslation()));
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Pose Jumped", Math.hypot(state.Pose.getTranslation().minus(lastPose.getTranslation()).getX(), state.Pose.getTranslation().minus(lastPose.getTranslation()).getY()) > (kSpeedAt12Volts.in(MetersPerSecond) * 2 * (Utils.getSystemTimeSeconds() - lastTime)));
		FIELD.setRobotPose(lastPose);

		lastPose = state.Pose;
		lastTime = Utils.getSystemTimeSeconds();
	}

	private void resetPID() {
		Pose2d currentPose = getPose();
		ChassisSpeeds currentSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), currentPose.getRotation());
		translationalController.reset(currentPose.getTranslation().getDistance(currentPose.getTranslation()), Math.min(0.0, -new Translation2d(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond).rotateBy(currentPose.getTranslation().getAngle().unaryMinus()).getX()));
	}

	@Override
	protected void stateExit() {
		resetPID();
	}

	public void setSOTMTarget(Pose2d targetPose) {
		sotmTarget = targetPose;
	}

	public List<TalonFX> getDriveMotors() {
		List<TalonFX> driveMotors = new java.util.ArrayList<>();
		for (int modules = 0; modules < driveIO.getDrive().getModules().length; modules++) {
			driveMotors.addAll(java.util.Arrays.asList(driveIO.getDrive().getModules()[modules].getDriveMotor()));
		}
		return driveMotors;
	}

	public List<TalonFX> getTurnMotors() {
		List<TalonFX> turnMotors = new java.util.ArrayList<>();
		for (int modules = 0; modules < driveIO.getDrive().getModules().length; modules++) {
			turnMotors.addAll(java.util.Arrays.asList(driveIO.getDrive().getModules()[modules].getSteerMotor()));
		}
		return turnMotors;
	}

	public boolean isAtAllianceShootingPosition() {
		return true;
		// var allianceOpt = DriverStation.getAlliance();
		// if (allianceOpt.isEmpty()) {
		// 	// Alliance not yet known (e.g., early init). Treat as not at alliance shooting position.
		// 	return false;
		// }
		// Alliance alliance = allianceOpt.get();
		// if (alliance == Alliance.Red) {
		// 	return getPose().getTranslation().getX() > ALLIANCE_SHOOTING_POSITION_THRESHOLD_RED.in(Meters);
		// } else {
		// 	return getPose().getTranslation().getX() < -ALLIANCE_SHOOTING_POSITION_THRESHOLD_BLUE.in(Meters);
		// }
	}
}
