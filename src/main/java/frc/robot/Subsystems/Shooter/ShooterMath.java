package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Subsystems.Shooter.ShooterConstants.ShotSampleData;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public final class ShooterMath {

	private static final List<ShotSampleData> HUB_TABLE = new ArrayList<>();
	private static final List<ShotSampleData> ALLIANCE_TABLE = new ArrayList<>();

	static {
		// Example data only — replace with real measured shots in ShooterConstants
		HUB_TABLE.addAll(HUB_SHOT_SAMPLES);

		ALLIANCE_TABLE.addAll(ALLIANCE_SHOT_SAMPLES);

		HUB_TABLE.sort(Comparator.comparingDouble(s -> s.distanceMeters()));
		ALLIANCE_TABLE.sort(Comparator.comparingDouble(s -> s.distanceMeters()));
	}

	public static Optional<ShotSolution> solveHubShot(Pose2d robotPose, Translation2d robotVelocity) {
		return solveMovingShot(robotPose, HUB_POSITION, robotVelocity, HUB_TABLE);
	}

	public static Optional<ShotSolution> solveAllianceShot(Pose2d robotPose, Translation2d robotVelocity) {
		return solveMovingShot(robotPose, ALLIANCE_BASKET_POSITION, robotVelocity, ALLIANCE_TABLE);
	}

	// Stolen Maths fr
	private static Optional<ShotSolution> solveMovingShot(Pose2d robotPose, Pose2d targetPose, Translation2d robotVelocity, List<ShotSampleData> table) {
		if (table.isEmpty()) {
			return Optional.empty();
		}

		Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());

		double baseDistance = toTarget.getNorm();
		if (baseDistance < SOLVER_EPSILON) {
			return Optional.empty();
		}

		Translation2d unitToTarget = toTarget.div(baseDistance);

		// Velocity component toward the target (m/s)
		double vParallel = robotVelocity.getX() * unitToTarget.getX() + robotVelocity.getY() * unitToTarget.getY();

		double predictedDistance = baseDistance;

		// Fixed-point iteration (converges quickly)
		for (int i = 0; i < SOLVER_ITERATIONS; i++) {
			ShotSolution sol = interpolateByDistance(predictedDistance, table);

			double newDistance = baseDistance - vParallel * sol.timeOfFlightSeconds;

			predictedDistance = clampToTable(newDistance, table);
		}

		return Optional.of(interpolateByDistance(predictedDistance, table));
	}

	// Handles interpolation within the shot table might be bad but should work
	private static ShotSolution interpolateByDistance(double distanceMeters, List<ShotSampleData> table) {
		if (distanceMeters <= table.get(0).distanceMeters()) {
			return new ShotSolution(table.get(0).hoodAngle(), table.get(0).flywheelSpeed(), table.get(0).timeOfFlightSeconds());
		}

		if (distanceMeters >= table.get(table.size() - 1).distanceMeters()) {
			int last = table.size() - 1;
			return new ShotSolution(table.get(last).hoodAngle(), table.get(last).flywheelSpeed(), table.get(last).timeOfFlightSeconds());
		}

		ShotSampleData lower = table.get(0);
		ShotSampleData upper = table.get(0);

		for (ShotSampleData s : table) {
			if (s.distanceMeters() <= distanceMeters) lower = s;
			if (s.distanceMeters() >= distanceMeters) {
				upper = s;
				break;
			}
		}

		double t = (distanceMeters - lower.distanceMeters()) / (upper.distanceMeters() - lower.distanceMeters());

		return new ShotSolution(
			Degrees.of(lerp(lower.hoodAngle().in(Degrees), upper.hoodAngle().in(Degrees), t)),
			RotationsPerSecond.of(lerp(lower.flywheelSpeed().in(RotationsPerSecond), upper.flywheelSpeed().in(RotationsPerSecond), t)),
			lerp(lower.timeOfFlightSeconds(), upper.timeOfFlightSeconds(), t)
		);
	}

	private static double clampToTable(double distance, List<ShotSampleData> table) {
		return Math.max(table.get(0).distanceMeters(), Math.min(distance, table.get(table.size() - 1).distanceMeters()));
	}

	private static double lerp(double a, double b, double t) {
		return a + (b - a) * t;
	}

	// A measured, real-world shot.
	// private record ShotSample(double distanceMeters, Angle hoodAngle, AngularVelocity flywheelSpeed, double timeOfFlightSeconds) {}

	// Solution returned by the solver.
	public record ShotSolution(Angle hoodAngle, AngularVelocity flywheelSpeed, double timeOfFlightSeconds) {
		// No-op: construction is done directly from ShotSampleData where needed.
	}
}
