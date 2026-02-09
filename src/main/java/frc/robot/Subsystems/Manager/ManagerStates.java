package frc.robot.Subsystems.Manager;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.GlobalConstants.FIELD_WIDTH;
import org.team7525.subsystem.SubsystemStates;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Subsystems.Climber.ClimberStates;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Hopper.HopperStates;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Shooter.ShooterStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("IDLE", IntakeStates.IN, HopperStates.IDLE, () -> ShooterStates.IDLE, ClimberStates.IDLE),
	EXTENDED_IDLE("EXTENDED_IDLE", IntakeStates.OUT, HopperStates.IDLE, () -> ShooterStates.IDLE, ClimberStates.IDLE),
	INTAKING("INTAKING", IntakeStates.INTAKE, HopperStates.SPINDEXING, () -> ShooterStates.STANDBY, ClimberStates.IDLE),
	WINDING_UP("WINDING_UP", IntakeStates.IN, HopperStates.IDLE, null, ClimberStates.IDLE),
	WINDING_UP_FIXED_SHOT("WINDING_UP_FIXED_SHOT", IntakeStates.IN, HopperStates.IDLE, () -> ShooterStates.SHOOT_FIXED, ClimberStates.IDLE),
	SHUTTLING("SHUTTLING", IntakeStates.IN, HopperStates.SPINDEXING, () -> shuttleStateSupplier(), ClimberStates.IDLE),
	SHOOTING_HUB("SHOOTING_HUB", IntakeStates.IN, HopperStates.SPINDEXING,  () -> ShooterStates.SHOOT_HUB, ClimberStates.IDLE),
	SHOOTING_FIXED("SHOOTING_FIXED", IntakeStates.IN, HopperStates.SPINDEXING, () -> ShooterStates.SHOOT_FIXED, ClimberStates.IDLE),
	EXTENDING_CLIMBER("EXTENDING_CLIMBER", IntakeStates.IN, HopperStates.IDLE, () -> ShooterStates.IDLE, ClimberStates.EXTEND),
	RETRACTING_CLIMBER("RETRACTING_CLIMBER", IntakeStates.IN, HopperStates.IDLE, () -> ShooterStates.IDLE, ClimberStates.RETRACT);

	private final String stateString;
	private final IntakeStates intakeState;
	private final HopperStates hopperState;
	private final Supplier<ShooterStates> shooterStateSupplier;
	private final ClimberStates climberState;

	ManagerStates(String stateString, IntakeStates intakeState, HopperStates hopperState, Supplier<ShooterStates> shooterState, ClimberStates climberState) {
		this.stateString = stateString;
		this.intakeState = intakeState;
		this.hopperState = hopperState;
		this.shooterStateSupplier = shooterState;
		this.climberState = climberState;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public IntakeStates getIntakeState() {
		return intakeState;
	}

	public HopperStates getHopperState() {
		return hopperState;
	}

	public Supplier<ShooterStates> getShooterStateSupplier() {
		return shooterStateSupplier;
	}

	public ClimberStates getClimberState() {
		return climberState;
	}

	private static ShooterStates shuttleStateSupplier() {
		Pose2d currentPose = Drive.getInstance().getPose();
		boolean onRightSide = currentPose.getMeasureY().in(Meters) > FIELD_WIDTH/2;
		onRightSide = Robot.isRedAlliance ? onRightSide : !onRightSide;

		if (onRightSide) {
			if (blockedByHub(currentPose, Robot.rightDeep)) return ShooterStates.SHOOT_ALLIANCE_RIGHT_DEEP;
			if (blockedByHub(currentPose, Robot.leftDeep)) return ShooterStates.SHOOT_ALLIANCE_LEFT_DEEP;
			if (blockedByHub(currentPose, Robot.rightShallow)) return ShooterStates.SHOOT_ALLIANCE_RIGHT_SHALLOW;
			if (blockedByHub(currentPose, Robot.rightShallow)) return ShooterStates.SHOOT_ALLIANCE_LEFT_SHALLOW;
			return ShooterStates.SHOOT_ALLIANCE_RIGHT_DEEP;
		} else {
			if (blockedByHub(currentPose, Robot.leftDeep)) return ShooterStates.SHOOT_ALLIANCE_LEFT_DEEP;
			if (blockedByHub(currentPose, Robot.rightDeep)) return ShooterStates.SHOOT_ALLIANCE_RIGHT_DEEP;
			if (blockedByHub(currentPose, Robot.rightShallow)) return ShooterStates.SHOOT_ALLIANCE_LEFT_SHALLOW;
			if (blockedByHub(currentPose, Robot.rightShallow)) return ShooterStates.SHOOT_ALLIANCE_RIGHT_SHALLOW;
			return ShooterStates.SHOOT_ALLIANCE_LEFT_DEEP;
		}
	}

	private static boolean blockedByHub(Pose2d shooterPose, Pose2d targetPose) {
		Pose2d closestPose = calculateClosestPoint(shooterPose, targetPose);

		if (closestPose.getX() > Robot.hubRegion_TL.getX() && closestPose.getX() < Robot.hubRegion_BR.getX()) return true;
		if (closestPose.getY() < Robot.hubRegion_TL.getY() && closestPose.getY() > Robot.hubRegion_BR.getY()) return true;
		return false;
	}

	private static Pose2d calculateClosestPoint(Pose2d startPose, Pose2d endPose) {
		double numeratorX = (Robot.hubPose.getX() - startPose.getX()) * (endPose.getX() - startPose.getX());
		double numeratorY = (Robot.hubPose.getY() - startPose.getY()) * (endPose.getY() - startPose.getY());
		double denominator = Math.pow(endPose.getX() - startPose.getX(), 2) + Math.pow(endPose.getY() - startPose.getY(), 2);

		return startPose.interpolate(endPose,(numeratorX + numeratorY) / denominator);
	}
}
