package frc.robot.Subsystems.Shooter.ShotSolver;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;
import static frc.robot.Subsystems.Shooter.ShotSolver.ShotTables.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
// Optional usages removed; methods now return null on failure

/**
 * Shooter ballistics solver with motion compensation.
 * 
 * Key features:
 * - Unified virtual target approach (handles all robot motion together)
 * - Iterative solver for time-of-flight prediction
 * - Three fixed-speed tables for simplified interpolation
 * - Shot confidence metrics
 * - Fast feasibility checks
 * - Safety limits
 * - Comprehensive logging
 */
public final class ShooterMath {

	// Shot tables
	private static final List<ShotSample> NEAR_TABLE = new ArrayList<>();
	private static final List<ShotSample> MEDIUM_TABLE = new ArrayList<>();
	private static final List<ShotSample> FAR_TABLE = new ArrayList<>();

	static {
		initializeTables();
	}

	private static void initializeTables() {
		if (NEAR_SHOT_SAMPLES != null && !NEAR_SHOT_SAMPLES.isEmpty()) {
			NEAR_TABLE.addAll(NEAR_SHOT_SAMPLES);
			NEAR_TABLE.sort(Comparator.comparingDouble(ShotSample::distance));
			validateTable(NEAR_TABLE, "NEAR");
		}

		if (MEDIUM_SHOT_SAMPLES != null && !MEDIUM_SHOT_SAMPLES.isEmpty()) {
			MEDIUM_TABLE.addAll(MEDIUM_SHOT_SAMPLES);
			MEDIUM_TABLE.sort(Comparator.comparingDouble(ShotSample::distance));
			validateTable(MEDIUM_TABLE, "MEDIUM");
		}

		if (FAR_SHOT_SAMPLES != null && !FAR_SHOT_SAMPLES.isEmpty()) {
			FAR_TABLE.addAll(FAR_SHOT_SAMPLES);
			FAR_TABLE.sort(Comparator.comparingDouble(ShotSample::distance));
			validateTable(FAR_TABLE, "FAR");
		}
	}

	private static void validateTable(List<ShotSample> table, String name) {
		if (table.isEmpty()) {
			Logger.recordOutput("Shooter/Validation/" + name + "_Empty", true);
			System.err.println("WARNING: " + name + " table is empty");
			return;
		}
		
		Logger.recordOutput("Shooter/Validation/" + name + "_Size", table.size());
		Logger.recordOutput("Shooter/Validation/" + name + "_MinDistance", table.get(0).distance());
		Logger.recordOutput("Shooter/Validation/" + name + "_MaxDistance", 
			table.get(table.size() - 1).distance());
		
		for (int i = 0; i < table.size() - 1; i++) {
			ShotSample curr = table.get(i);
			ShotSample next = table.get(i + 1);
			
			if (Math.abs(next.distance() - curr.distance()) < DISTANCE_EPSILON) {
				System.err.println("WARNING: " + name + " has duplicate distances at index " + i);
			}
			if (curr.timeOfFlight() <= 0) {
				System.err.println("WARNING: " + name + " has invalid TOF at index " + i);
			}
			if (curr.hoodAngle().in(Degrees) < 0 || curr.hoodAngle().in(Degrees) > 90) {
				System.err.println("WARNING: " + name + " has suspicious angle at index " + i);
			}
		}
	}

	// ========== PUBLIC API ==========

	/**
	 * Solve for shot to custom target.
	 * 
	 * @param robotPose Current robot position and orientation
	 * @param robotVelocity Robot velocity in field coordinates (m/s)
	 * @param targetPose Target position to shoot at
	 * @return Shot solution or empty if shot is not feasible
	 */
	public static ShotSolution solveShot(
			Pose2d robotPose,
			Translation2d robotVelocity,
			Pose2d targetPose) {
		
		long startNanos = System.nanoTime();
		
		Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
		double baseDistance = toTarget.getNorm();
		
		Logger.recordOutput("Shooter/Solve/BaseDistance", baseDistance);
		Logger.recordOutput("Shooter/Solve/VelocityMag", robotVelocity.getNorm());
		
		if (baseDistance < DISTANCE_EPSILON) {
			Logger.recordOutput("Shooter/Solve/Error", "Robot on target");
			Logger.recordOutput("Shooter/Solve/Success", false);
			return null;
		}

		ShotTable table = selectTable(baseDistance);
		if (table.samples.isEmpty()) {
			Logger.recordOutput("Shooter/Solve/Error", "No valid table");
			Logger.recordOutput("Shooter/Solve/Success", false);
			return null;
		}
		
		Logger.recordOutput("Shooter/Solve/Table", table.name);

	ShotSolution solution = solve(robotPose, targetPose, robotVelocity, table);
		
		double timeMs = (System.nanoTime() - startNanos) / 1_000_000.0;
		Logger.recordOutput("Shooter/Solve/TimeMs", timeMs);
	Logger.recordOutput("Shooter/Solve/Success", solution != null);
		
		return solution;
	}

	/**
	 * Fast feasibility check for custom target.
	 * 
	 * @param robotPose Current robot position
	 * @param targetPose Target pose to check
	 * @return True if target is within any table's range
	 */
	public static boolean canReachTarget(Pose2d robotPose, Pose2d targetPose) {
		double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
		
		boolean reachable = isInRange(distance, NEAR_TABLE) ||
		                    isInRange(distance, MEDIUM_TABLE) ||
		                    isInRange(distance, FAR_TABLE);
		
		Logger.recordOutput("Shooter/Feasibility/Distance", distance);
		Logger.recordOutput("Shooter/Feasibility/Reachable", reachable);
		
		return reachable;
	}

	// ========== SOLVER ==========

	private static ShotSolution solve(
			Pose2d robotPose,
			Pose2d targetPose,
			Translation2d robotVelocity,
			ShotTable table) {
		
		Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
		double baseDistance = toTarget.getNorm();

		Logger.recordOutput("Shooter/Solve/VelX", robotVelocity.getX());
		Logger.recordOutput("Shooter/Solve/VelY", robotVelocity.getY());

		double predictedDistance = baseDistance;
		Translation2d virtualOffset = new Translation2d();
		int iterations = 0;

		// ITERATIVE SOLVER
		// Why iterate? We need TOF to predict distance, but we need distance to get TOF.
		// This loop converges on the correct distance accounting for robot motion.
		for (int i = 0; i < SOLVER_ITERATIONS; i++) {
			iterations = i + 1;
			
			// Get shot parameters for current predicted distance
			ShotParams params = interpolate(predictedDistance, table.samples);
			if (params == null) {
				double clamped = clamp(predictedDistance, table.samples);
				params = interpolate(clamped, table.samples);
				if (params == null) {
					Logger.recordOutput("Shooter/Solve/Error", "Interpolation failed");
					return null;
				}
				predictedDistance = clamped;
			}
			
			ShotParams p = params;

			// UNIFIED VIRTUAL TARGET
			// Game piece inherits robot's full velocity (both x and y components)
			// During flight, it drifts by: velocity * time-of-flight
			// Compensate by aiming at virtual target shifted opposite to drift
			Translation2d drift = robotVelocity.times(p.timeOfFlight);
			virtualOffset = drift.unaryMinus();  // Opposite direction
			
			// Calculate distance to virtual target
			Translation2d virtualPos = toTarget.plus(virtualOffset);
			double newDistance = virtualPos.getNorm();

			Logger.recordOutput("Shooter/Solve/Iter" + i + "_Dist", predictedDistance);

			// Check convergence
			if (Math.abs(newDistance - predictedDistance) < CONVERGENCE_THRESHOLD) {
				predictedDistance = newDistance;
				break;
			}

			predictedDistance = newDistance;
		}
		
		Logger.recordOutput("Shooter/Solve/Iterations", iterations);
		Logger.recordOutput("Shooter/Solve/FinalDist", predictedDistance);
		Logger.recordOutput("Shooter/Solve/VirtualOffsetX", virtualOffset.getX());
		Logger.recordOutput("Shooter/Solve/VirtualOffsetY", virtualOffset.getY());

		// Final solution
		double finalDist = clamp(predictedDistance, table.samples);
		ShotParams finalParams = interpolate(finalDist, table.samples);
		
		if (finalParams == null) {
			Logger.recordOutput("Shooter/Solve/Error", "Final interp failed");
			return null;
		}
		
		ShotParams p = finalParams;
		
		// Safety check
		if (!isAngleSafe(p.hoodAngle)) {
			Logger.recordOutput("Shooter/Solve/Error", "Unsafe angle");
			Logger.recordOutput("Shooter/Solve/UnsafeAngle", p.hoodAngle.in(Degrees));
			return null;
		}

		// Build solution
		Pose2d virtualTarget = new Pose2d(
			targetPose.getTranslation().plus(virtualOffset),
			targetPose.getRotation()
		);
		
		double confidence = calcConfidence(finalDist, table, robotVelocity.getNorm(), iterations);
		
		Logger.recordOutput("Shooter/Solve/Confidence", confidence);
		Logger.recordOutput("Shooter/Solve/HoodAngle", p.hoodAngle.in(Degrees));
		Logger.recordOutput("Shooter/Solve/FlywheelSpeed", table.flywheelSpeed.in(RotationsPerSecond));
		Logger.recordOutput("Shooter/Solve/TOF", p.timeOfFlight);

		return new ShotSolution(
			p.hoodAngle,
			table.flywheelSpeed,
			virtualTarget,
			p.timeOfFlight,
			confidence,
			new DebugInfo(table.name, iterations, finalDist, virtualOffset)
		);
	}

	// ========== TABLE SELECTION ==========

	private static ShotTable selectTable(double dist) {
		boolean nearOk = isInRange(dist, NEAR_TABLE);
		boolean medOk = isInRange(dist, MEDIUM_TABLE);
		boolean farOk = isInRange(dist, FAR_TABLE);

		// Multiple valid - choose closest to center
		if (nearOk && medOk && farOk) {
			double nearD = distToCenter(dist, NEAR_TABLE);
			double medD = distToCenter(dist, MEDIUM_TABLE);
			double farD = distToCenter(dist, FAR_TABLE);
			
			if (nearD <= medD && nearD <= farD) {
				return new ShotTable(NEAR_TABLE, NEAR_FLYWHEEL_SPEED, "NEAR");
			} else if (medD <= farD) {
				return new ShotTable(MEDIUM_TABLE, MEDIUM_FLYWHEEL_SPEED, "MEDIUM");
			} else {
				return new ShotTable(FAR_TABLE, FAR_FLYWHEEL_SPEED, "FAR");
			}
		}
		
		if (nearOk) return new ShotTable(NEAR_TABLE, NEAR_FLYWHEEL_SPEED, "NEAR");
		if (medOk) return new ShotTable(MEDIUM_TABLE, MEDIUM_FLYWHEEL_SPEED, "MEDIUM");
		if (farOk) return new ShotTable(FAR_TABLE, FAR_FLYWHEEL_SPEED, "FAR");
		
		// Out of range - use nearest
		if (!NEAR_TABLE.isEmpty() && dist < NEAR_TABLE.get(0).distance()) {
			Logger.recordOutput("Shooter/Solve/Warning", "Below NEAR range");
			return new ShotTable(NEAR_TABLE, NEAR_FLYWHEEL_SPEED, "NEAR");
		}
		if (!FAR_TABLE.isEmpty() && dist > FAR_TABLE.get(FAR_TABLE.size() - 1).distance()) {
			Logger.recordOutput("Shooter/Solve/Warning", "Above FAR range");
			return new ShotTable(FAR_TABLE, FAR_FLYWHEEL_SPEED, "FAR");
		}
		
		// Fallback
		if (!NEAR_TABLE.isEmpty()) return new ShotTable(NEAR_TABLE, NEAR_FLYWHEEL_SPEED, "NEAR");
		if (!MEDIUM_TABLE.isEmpty()) return new ShotTable(MEDIUM_TABLE, MEDIUM_FLYWHEEL_SPEED, "MEDIUM");
		if (!FAR_TABLE.isEmpty()) return new ShotTable(FAR_TABLE, FAR_FLYWHEEL_SPEED, "FAR");
		
		return new ShotTable(new ArrayList<>(), RotationsPerSecond.of(0), "NONE");
	}

	private static boolean isInRange(double dist, List<ShotSample> table) {
		if (table.isEmpty()) return false;
		return dist >= table.get(0).distance() && dist <= table.get(table.size() - 1).distance();
	}

	private static double distToCenter(double dist, List<ShotSample> table) {
		double center = (table.get(0).distance() + table.get(table.size() - 1).distance()) / 2.0;
		return Math.abs(dist - center);
	}

	// ========== INTERPOLATION ==========

	private static ShotParams interpolate(double dist, List<ShotSample> table) {
		if (table == null || table.isEmpty()) return null;
		if (dist < table.get(0).distance() - DISTANCE_EPSILON) return null;
		if (dist > table.get(table.size() - 1).distance() + DISTANCE_EPSILON) return null;

		// Exact matches
		if (Math.abs(dist - table.get(0).distance()) < DISTANCE_EPSILON) {
			ShotSample s = table.get(0);
			return new ShotParams(s.hoodAngle(), s.timeOfFlight());
		}
		
		int last = table.size() - 1;
		if (Math.abs(dist - table.get(last).distance()) < DISTANCE_EPSILON) {
			ShotSample s = table.get(last);
			return new ShotParams(s.hoodAngle(), s.timeOfFlight());
		}

		// Binary search
		int lower = binarySearchFloor(table, dist);
		int upper = lower + 1;

		if (upper >= table.size()) {
			ShotSample s = table.get(last);
			return new ShotParams(s.hoodAngle(), s.timeOfFlight());
		}

		ShotSample lo = table.get(lower);
		ShotSample hi = table.get(upper);

		double range = hi.distance() - lo.distance();
		if (range < DISTANCE_EPSILON) {
			return new ShotParams(lo.hoodAngle(), lo.timeOfFlight());
		}
		
		double t = (dist - lo.distance()) / range;

		return new ShotParams(
			Degrees.of(lerp(lo.hoodAngle().in(Degrees), hi.hoodAngle().in(Degrees), t)),
			lerp(lo.timeOfFlight(), hi.timeOfFlight(), t)
		);
	}

	private static int binarySearchFloor(List<ShotSample> table, double dist) {
		int left = 0, right = table.size() - 1, result = 0;
		while (left <= right) {
			int mid = left + (right - left) / 2;
			if (table.get(mid).distance() <= dist) {
				result = mid;
				left = mid + 1;
			} else {
				right = mid - 1;
			}
		}
		return result;
	}

	// ========== SAFETY ==========

	private static boolean isAngleSafe(Angle angle) {
		double deg = angle.in(Degrees);
		return deg >= HOOD_MIN_ANGLE.in(Degrees) && deg <= HOOD_MAX_ANGLE.in(Degrees);
	}

	// ========== CONFIDENCE ==========

	private static double calcConfidence(double dist, ShotTable table, double velMag, int iters) {
		if (table.samples.isEmpty()) return 0.0;
		
		double min = table.samples.get(0).distance();
		double max = table.samples.get(table.samples.size() - 1).distance();
		double center = (min + max) / 2.0;
		double range = max - min;
		
		// Distance factor: higher confidence near table center
		double distConf = 1.0 - Math.min(1.0, Math.abs(dist - center) / (range / 2.0));
		
		// Velocity factor: lower velocity = higher confidence
		double velConf = 1.0 - Math.min(1.0, velMag / 3.0);
		
		// Convergence factor: faster convergence = higher confidence
		double convConf = 1.0 - (0.5 * (iters - 1) / (SOLVER_ITERATIONS - 1));
		
		double conf = 0.5 * distConf + 0.3 * velConf + 0.2 * convConf;
		
		Logger.recordOutput("Shooter/Confidence/Dist", distConf);
		Logger.recordOutput("Shooter/Confidence/Vel", velConf);
		Logger.recordOutput("Shooter/Confidence/Conv", convConf);
		
		return Math.max(0.0, Math.min(1.0, conf));
	}

	// ========== UTILITY ==========

	private static double clamp(double dist, List<ShotSample> table) {
		if (table.isEmpty()) return dist;
		return Math.max(table.get(0).distance(), 
		                Math.min(dist, table.get(table.size() - 1).distance()));
	}

	private static double lerp(double a, double b, double t) {
		return a + (b - a) * t;
	}

	// ========== INNER CLASSES ==========

	private static record ShotTable(List<ShotSample> samples, AngularVelocity flywheelSpeed, String name) {}
	
	private static record ShotParams(Angle hoodAngle, double timeOfFlight) {}
}