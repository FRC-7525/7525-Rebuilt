package frc.robot.Subsystems.Shooter;

import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.ShotSolver.ShooterMath;
import frc.robot.Subsystems.Shooter.ShotSolver.ShotSolution;

import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates;

public enum ShooterStates implements SubsystemStates {
	IDLE("IDLE", () -> null),
	REVERSE("REVERSE", () -> new ShotSolution(REVERSE_SHOT_ANGLE, REVERSE_SHOT_SPEED, Drive.getInstance().getPose(), 0.0, 1.0, null)), // TODO: get good value
	// Use placeholder Pose2d and zero velocity for now; replace with real robot pose/velocity when available.
	SHOOT_HUB("SHOOT HUB", () -> ShooterMath.solveShot(Drive.getInstance().getPose(), Drive.getInstance().getVelocityTranslationFieldRelative(), BLUE_HUB_POSE)),
	SHOOT_ALLIANCE("SHOOT ALLIANCE", () -> ShooterMath.solveShot(Drive.getInstance().getPose(), Drive.getInstance().getVelocityTranslationFieldRelative(), SHALLOW_ALLIANCE_POSE)),
	SHOOT_ALLIANCE_FAR("SHOOT ALLIANCE FAR", () -> ShooterMath.solveShot(Drive.getInstance().getPose(), Drive.getInstance().getVelocityTranslationFieldRelative(), DEEP_ALLIANCE_POSE)),
	SHOOT_FIXED("SHOOT FIXED", () -> new ShotSolution(FIXED_SHOT_ANGLE, FIXED_SHOT_SPEED, Drive.getInstance().getPose(), 0.0, 1.0, null)),
	STANDBY("STANDBY", () -> new ShotSolution(STANDBY_ANGLE, STANDBY_SPEED, Drive.getInstance().getPose(), 0.0, 1.0, null));

	private String stateString;
	private Supplier<ShotSolution> shotSolutionSupplier;

	private ShooterStates(String stateString, Supplier<ShotSolution> shotSolutionSupplier) {
		this.stateString = stateString;
		this.shotSolutionSupplier = shotSolutionSupplier;
	}

	public String getStateString() {
		return stateString;
	}

	public ShotSolution getShotSolution() {
		return shotSolutionSupplier.get();
	}
}
