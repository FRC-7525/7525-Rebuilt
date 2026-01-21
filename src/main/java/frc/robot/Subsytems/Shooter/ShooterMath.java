package frc.robot.Subsytems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsytems.Shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public final class ShooterMath {

	private static final List<ShotSample> HUB_TABLE = new ArrayList<>();
	private static final List<ShotSample> ALLIANCE_TABLE = new ArrayList<>();

	static {
		// Example data only — replace with real measured shots
		HUB_TABLE.add(new ShotSample(2.0, Degrees.of(30), RotationsPerSecond.of(20), 0.50));
		HUB_TABLE.add(new ShotSample(3.0, Degrees.of(35), RotationsPerSecond.of(25), 0.55));
		HUB_TABLE.add(new ShotSample(4.0, Degrees.of(40), RotationsPerSecond.of(30), 0.60));
		HUB_TABLE.add(new ShotSample(5.0, Degrees.of(45), RotationsPerSecond.of(35), 0.65));

		ALLIANCE_TABLE.add(new ShotSample(2.0, Degrees.of(28), RotationsPerSecond.of(19), 0.52));
		ALLIANCE_TABLE.add(new ShotSample(3.0, Degrees.of(33), RotationsPerSecond.of(24), 0.56));
		ALLIANCE_TABLE.add(new ShotSample(4.0, Degrees.of(38), RotationsPerSecond.of(29), 0.61));
		ALLIANCE_TABLE.add(new ShotSample(5.0, Degrees.of(44), RotationsPerSecond.of(34), 0.66));

		HUB_TABLE.sort(Comparator.comparingDouble(s -> s.distanceMeters));
		ALLIANCE_TABLE.sort(Comparator.comparingDouble(s -> s.distanceMeters));
	}

	public static Optional<ShotSolution> solveHubShot(Pose2d robotPose, Translation2d robotVelocity) {
		return solveMovingShot(robotPose, HUB_POSITION, robotVelocity, HUB_TABLE);
	}

	public static Optional<ShotSolution> solveAllianceShot(Pose2d robotPose, Translation2d robotVelocity) {
		return solveMovingShot(robotPose, ALLIANCE_BASKET_POSITION, robotVelocity, ALLIANCE_TABLE);
	}

	/**
	 * Stollen Math
	 * - Compute initial distance to target
	 * - Project robot velocity onto target direction
	 * - Predict distance at impact:
	 *
	 *     d_impact = d_now - v_parallel * time_of_flight
	 *
	 * - Iterate a few times to converge
	 */
	private static Optional<ShotSolution> solveMovingShot(Pose2d robotPose, Pose2d targetPose, Translation2d robotVelocity, List<ShotSample> table) {
		if (table.isEmpty()) {
			return Optional.empty();
		}

		Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());

		double baseDistance = toTarget.getNorm();
		if (baseDistance < 1e-6) {
			return Optional.empty();
		}

		Translation2d unitToTarget = toTarget.div(baseDistance);

		// Velocity component toward the target (m/s)
		double vParallel = robotVelocity.getX() * unitToTarget.getX() + robotVelocity.getY() * unitToTarget.getY();

		double predictedDistance = baseDistance;

		// Fixed-point iteration (converges quickly)
		for (int i = 0; i < 4; i++) {
			ShotSolution sol = interpolateByDistance(predictedDistance, table);

			double newDistance = baseDistance - vParallel * sol.timeOfFlightSeconds;

			predictedDistance = clampToTable(newDistance, table);
		}

		return Optional.of(interpolateByDistance(predictedDistance, table));
	}

	// Handles interpolation within the shot table might be bad but should work
	private static ShotSolution interpolateByDistance(double distanceMeters, List<ShotSample> table) {
		if (distanceMeters <= table.get(0).distanceMeters) {
			return ShotSolution.from(table.get(0));
		}

		if (distanceMeters >= table.get(table.size() - 1).distanceMeters) {
			return ShotSolution.from(table.get(table.size() - 1));
		}

		ShotSample lower = table.get(0);
		ShotSample upper = table.get(0);

		for (ShotSample s : table) {
			if (s.distanceMeters <= distanceMeters) lower = s;
			if (s.distanceMeters >= distanceMeters) {
				upper = s;
				break;
			}
		}

		double t = (distanceMeters - lower.distanceMeters) / (upper.distanceMeters - lower.distanceMeters);

		return new ShotSolution(Degrees.of(lerp(lower.hoodAngle.in(Degrees), upper.hoodAngle.in(Degrees), t)), RotationsPerSecond.of(lerp(lower.flywheelSpeed.in(RotationsPerSecond), upper.flywheelSpeed.in(RotationsPerSecond), t)), lerp(lower.timeOfFlightSeconds, upper.timeOfFlightSeconds, t));
	}

	private static double clampToTable(double distance, List<ShotSample> table) {
		return Math.max(table.get(0).distanceMeters, Math.min(distance, table.get(table.size() - 1).distanceMeters));
	}

	private static double lerp(double a, double b, double t) {
		return a + (b - a) * t;
	}

	// A measured, real-world shot.
	private record ShotSample(double distanceMeters, Angle hoodAngle, AngularVelocity flywheelSpeed, double timeOfFlightSeconds) {}

	// Solution returned by the solver.
	public record ShotSolution(Angle hoodAngle, AngularVelocity flywheelSpeed, double timeOfFlightSeconds) {
		private static ShotSolution from(ShotSample s) {
			return new ShotSolution(s.hoodAngle, s.flywheelSpeed, s.timeOfFlightSeconds);
		}
	}
	/* ------------------------------------------------------------------
	 * SHOT TABLE TUNING GUIDE
	 * ------------------------------------------------------------------
	 * HOW TO BUILD THE SHOT TABLE:
	 *
	 * 1. Disable interpolation and motion compensation.
	 * 2. Place robot at a known distance (use odometry or tape measure).
	 * 3. Tune hood angle and flywheel speed until shots are consistent.
	 * 4. Measure time of flight:
	 *      - High-speed camera, or
	 *      - Time from feeder release to score sensor
	 * 5. Add one entry per ~0.5–1.0 meters.
	 *
	 */
}
