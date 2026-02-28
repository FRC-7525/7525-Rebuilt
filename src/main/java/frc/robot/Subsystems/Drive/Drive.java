package frc.robot.Subsystems.Drive;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants.*;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import static frc.robot.Subsystems.Drive.TunerConstants.*;
import static frc.robot.Subsystems.Shooter.ShooterConstants.ROBOT_TO_SHOOTER;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.GlobalConstants;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.AutoAlign.AutoAlignConstants.Obstacles;
import frc.robot.Subsystems.Drive.AutoAlign.MathHelpers;
import frc.robot.Subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Subsystems.Manager.Manager;
import frc.robot.Subsystems.Manager.ManagerStates;
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
	private final ProfiledPIDController shooterYawController;
	private final SimpleMotorFeedforward shooterYawFeedforward;
	private final PIDController repulsorTranslationController;
	private final PIDController repulsorRotationalController;
	private final SlewRateLimiter xRateLimiter;
	private final SlewRateLimiter yRateLimiter;
	// Aim smoothing and angular-rate limiting for AIMLOCK case
	private Translation2d smoothedFieldVel = new Translation2d(0.0, 0.0);
	private double aimLookaheadSeconds = 0.05; // small lookahead to predict shooter motion (s)

	private double driveErrorAbs;
	private double thetaErrorAbs;
	private double ffMinRadius = 0.2, ffMaxRadius = 1.0;
	private Angle aimError = Degrees.of(0);

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
		this.xRateLimiter = new SlewRateLimiter(2);
		this.yRateLimiter = new SlewRateLimiter(2);

		this.shooterYawFeedforward = SHOOTER_YAW_FEEDFORWARD.get();
		this.shooterYawController = SHOOTER_YAW_CONTROLLER.get();
		this.shooterYawController.setTolerance(ANGLE_ERROR_MARGIN.in(Radians));
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

		this.isFieldRelative = true;
		this.sotmTarget = Pose2d.kZero;

		// Zero Gyro
		addRunnableTrigger(
			() -> {
				driveIO.zeroGyro();
			},
			OPERATOR_CONTROLLER::getBackButtonPressed
		);
		addRunnableTrigger(() -> isFieldRelative = !isFieldRelative, DRIVER_CONTROLLER::getBackButtonPressed);
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

		// Responsible for shooting and stuff
		if (Manager.getInstance().getState() == ManagerStates.SHOOTING_HUB) {
			setState(DriveStates.AIMLOCK_HUB);
		} else if (getState() == DriveStates.AIMLOCK_HUB) {
			setState(DriveStates.NORMAL);
		}
		

		switch (getState()) {
			case NORMAL:
				executeDriveInstruction(-DRIVER_CONTROLLER.getLeftY() * kSpeedAt12Volts.in(MetersPerSecond), -DRIVER_CONTROLLER.getLeftX() * kSpeedAt12Volts.in(MetersPerSecond), -DRIVER_CONTROLLER.getRightX() * ANGULAR_VELOCITY_LIMIT.in(RadiansPerSecond) * 0.1, isFieldRelative);
				break;
			case AIMLOCK_ALLIANCE_LEFT_SHALLOW:
			case AIMLOCK_ALLIANCE_LEFT_DEEP:
			case AIMLOCK_ALLIANCE_RIGHT_DEEP:
			case AIMLOCK_ALLIANCE_RIGHT_SHALLOW:
			case AIMLOCK_HUB:
				// Compute shooter pose (robot -> shooter transform) and aim from the shooter, not robot center.
				// ROBOT_TO_SHOOTER is a 3D transform; extract the 2D translation and yaw rotation.

				// Variable Init:
				Pose2d robotPose = getPose();
				Translation3d shooterTranslation = ROBOT_TO_SHOOTER.getTranslation();
				Transform2d shooterOffset = new Transform2d(new Translation2d(shooterTranslation.getX(), shooterTranslation.getY()), Rotation2d.fromRadians(ROBOT_TO_SHOOTER.getRotation().getZ()));
				Pose2d shooterPose = robotPose.transformBy(shooterOffset);
				ChassisSpeeds fieldSpeeds = getFieldCentricSpeeds();
				Translation2d currentFieldVel = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

				// Exponentially smooth robot vel, smooths out angry accel that can mess up path of ball
				smoothedFieldVel = new Translation2d(
					(AIM_VEL_ALPHA * currentFieldVel.getX()) + ((1 - AIM_VEL_ALPHA) * smoothedFieldVel.getX()),
					(AIM_VEL_ALPHA * currentFieldVel.getY()) + ((1 - AIM_VEL_ALPHA) * smoothedFieldVel.getY())
				);
				
				// Peek ahead a little to infer shooter pos (ts runs a little after loop init)
				Translation2d predictedShooterPos = shooterPose.getTranslation().plus(new Translation2d(smoothedFieldVel.getX() * aimLookaheadSeconds, smoothedFieldVel.getY() * aimLookaheadSeconds));

				// Points from shooter to target/calcs
				var aimVector = sotmTarget.getTranslation().minus(predictedShooterPos);
				double targetDistance = aimVector.getNorm();
				Rotation2d targetFromShooter = aimVector.getAngle();

				// Desired robot heading so that the shooter (which has a fixed yaw offset) faces the target
				Rotation2d shooterYawOffset = Rotation2d.fromRadians(ROBOT_TO_SHOOTER.getRotation().getZ());
				Rotation2d desiredRobotHeading = targetFromShooter.minus(shooterYawOffset);

				double omegaRadiansPerSecond = calculateAngularVelocity(desiredRobotHeading.getMeasure()).in(RadiansPerSecond);

				// Rate-limit changes in commanded angular velocity to reduce jumpiness
				// omegaRadiansPerSecond = aimAngularRateLimiter.calculate(omegaRadiansPerSecond);
				executeAutoAlignDriveInstruction(
					yRateLimiter.calculate(-0.5 * DRIVER_CONTROLLER.getLeftY() * kSpeedAt12Volts.in(MetersPerSecond)),
					xRateLimiter.calculate(-0.5 * DRIVER_CONTROLLER.getLeftX() * kSpeedAt12Volts.in(MetersPerSecond)),
					omegaRadiansPerSecond,
					true,
					isFieldRelative
				);

				// LOGGING

				// Calcs:
				Translation2d shooterToTarget = aimVector;
				Rotation2d bearingToTarget = shooterToTarget.getAngle();
				Rotation2d shooterRotation = shooterPose.getRotation();
				Rotation2d shooterToTargetError = bearingToTarget.minus(shooterRotation);
				aimError = Degrees.of(shooterToTargetError.getDegrees());

				// Field
				field.getObject("Shooter").setPose(shooterPose);
				field.getObject("Target").setPose(sotmTarget);

				// Akit Outputs
				Logger.recordOutput("shooter/target", targetFromShooter);
				Logger.recordOutput("shooter/Angle Diff To Target", shooterToTargetError.getDegrees());
				Logger.recordOutput("shooter/ShooterPosition", shooterPose);
				Logger.recordOutput("shooter/TargetDistance", targetDistance);

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

		// Read and expose the distance slider on the SmartDashboard. Default to 1 meter.
		//double distance = SmartDashboard.getNumber("distance", 1.0);
		//SmartDashboard.putNumber("distance", distance);
		// Set the robot pose every loop to be the blue hub pose shifted left by distance on the X axis.
		// Subtract distance from the BLUE_HUB_POSE.x to move the pose along negative X.
		//Pose2d shiftedPose = new Pose2d(BLUE_HUB_POSE.getX() - distance, BLUE_HUB_POSE.getY(), Rotation2d.fromDegrees(-90));
		//getDriveTrain().resetPose(shiftedPose);
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

	public void executeAutoAlignDriveInstruction(double xVelocity, double yVelocity, double angularVelocity, boolean hasDriverControl, boolean fieldRelative) {
		if (hasDriverControl) {
			if (fieldRelative) {
				driveIO.setControl(
					new SwerveRequest.FieldCentric().withDeadband(DEADBAND).withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
				);
			} else {
				driveIO.setControl(
					new SwerveRequest.RobotCentric().withDeadband(DEADBAND).withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
				);
			}
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
			executeAutoAlignDriveInstruction(-translationVelocity.getX(), translationVelocity.getY(), thetaVelocity, false, false);
		} else {
			executeAutoAlignDriveInstruction(translationVelocity.getX(), translationVelocity.getY(), thetaVelocity, false, false);
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

	public ChassisSpeeds getFieldCentricSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(driveIO.getDrive().getState().Speeds, getPose().getRotation());
	}

	public Translation2d getVelocityTranslationFieldRelative() {
		ChassisSpeeds speeds = getFieldCentricSpeeds();
		return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
	}

	public Translation2d getTargetVector(Pose2d target) {
		// Thank you yagsl :)
		var currentPose = getPose();
		//var currentFieldOrientedSpeeds = getFieldCentricSpeeds();
		// if (aimLookaheadTime.isPresent())
		// {
		// var aimLookAhead = aimLookaheadTime.get().in(Seconds);
		// var poseTransform = new Transform2d(Meters.of(currentFieldOrientedSpeeds.vxMetersPerSecond * aimLookAhead),
		// 									Meters.of(currentFieldOrientedSpeeds.vyMetersPerSecond * aimLookAhead),
		// 									Rotation2d.kZero);
		// currentPose = currentPose.plus(poseTransform);
		// }
		return target.getTranslation().minus(currentPose.getTranslation());
  	}

	public AngularVelocity calculateAngularVelocity(Angle target) {
    	var omegaRadiansPerSecond = shooterYawController.calculate(getPose().getRotation().getRadians(), target.in(Radians)) * 
			MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) + 
			shooterYawFeedforward.calculate(shooterYawController.getSetpoint().velocity);
    	return RadiansPerSecond.of(omegaRadiansPerSecond);
	}



	public boolean isAtAllianceShootingPosition() {
		if (GlobalConstants.TESTING) {
			// In testing mode, allow testing the alliance shooting position logic without needing to be at the actual field positions.
			return true;
		}
		var allianceOpt = DriverStation.getAlliance();
		if (allianceOpt.isEmpty()) {
			return false;
		}
		Alliance alliance = allianceOpt.get();
		if (alliance == Alliance.Red) {
			return getPose().getTranslation().getX() > ALLIANCE_SHOOTING_POSITION_THRESHOLD_RED; // TODO: get value
		} else {
			return getPose().getTranslation().getX() < -ALLIANCE_SHOOTING_POSITION_THRESHOLD_BLUE; // TODO: get value
		}
	}

	public boolean isAtSOTMTarget() {
		return Math.abs(aimError.in(Degrees)) < ANGLE_ERROR_MARGIN.in(Degrees);
	}
}