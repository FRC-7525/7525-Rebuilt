package frc.robot.Subsystems.Shooter.ShotSolver;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShotSolver.ShotSolverConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import static frc.robot.Subsystems.Shooter.ShotSolver.ShotTables.*;

/**
 * Shooter solver implementing SOTF features (recursion, Newton option, latency, turret-tip velocity, etc.).
 */
public final class ShooterMath {

	private static final List<ShotSample> NEAR_TABLE = new ArrayList<>();
	private static final List<ShotSample> FAR_TABLE = new ArrayList<>();

	static {
		initializeTables();
	}

	private static void initializeTables() {
		if (NEAR_SHOT_SAMPLES != null && !NEAR_SHOT_SAMPLES.isEmpty()) {
			NEAR_TABLE.addAll(NEAR_SHOT_SAMPLES); // TODO: Sim checks should be added
			NEAR_TABLE.sort(Comparator.comparingDouble(ShotSample::distance));
			validateTable(NEAR_TABLE, "NEAR");
		}
		// MEDIUM table removed; use NEAR and FAR only
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
		Logger.recordOutput("Shooter/Validation/" + name + "_MaxDistance", table.get(table.size() - 1).distance());

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

	public static ShotSolution solveShot(Pose2d robotPose, Translation2d robotVelocity, Pose2d targetPose) {
		// Record which solver flags are active to aid debugging/tuning
		Logger.recordOutput("Shooter/Wrapper/UseNewton", ShotSolverConstants.USE_NEWTON);
		Logger.recordOutput("Shooter/Wrapper/UseEffectiveTOF", ShotSolverConstants.USE_EFFECTIVE_TOF);
		Logger.recordOutput("Shooter/Wrapper/UseLatencyComp", ShotSolverConstants.USE_LATENCY_COMPENSATION);
		Logger.recordOutput("Shooter/Wrapper/SensorLatencySeconds", ShotSolverConstants.SENSOR_LATENCY_SECONDS);

		return solveShot(robotPose, robotVelocity, targetPose, 0.0);
	}

	public static ShotSolution solveShot(Pose2d robotPose, Translation2d robotVelocity, Pose2d targetPose, double robotAngularVelocityRadPerSec) {
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

		ShotSolution solution = solve(robotPose, targetPose, robotVelocity, robotAngularVelocityRadPerSec, table);

		double timeMs = (System.nanoTime() - startNanos) / 1_000_000.0;
		Logger.recordOutput("Shooter/Solve/TimeMs", timeMs);
		Logger.recordOutput("Shooter/Solve/Success", solution != null);

		return solution;
	}

	public static boolean canReachTarget(Pose2d robotPose, Pose2d targetPose) {
	double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
	boolean reachable = isInRange(distance, NEAR_TABLE) || isInRange(distance, FAR_TABLE);
		Logger.recordOutput("Shooter/Feasibility/Distance", distance);
		Logger.recordOutput("Shooter/Feasibility/Reachable", reachable);
		return reachable;
	}

	// ========== SOLVER ==========

	private static ShotSolution solve(Pose2d robotPose, Pose2d targetPose, Translation2d robotVelocity, double robotAngularVelocityRadPerSec, ShotTable table) {
		Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
		double baseDistance = toTarget.getNorm();

		// Optionally compensate for sensor latency by projecting robot pose forward
		Translation2d compensatedOffset = new Translation2d();
		if (USE_LATENCY_COMPENSATION) {
			compensatedOffset = robotVelocity.times(SENSOR_LATENCY_SECONDS);
			toTarget = toTarget.minus(compensatedOffset);
			Logger.recordOutput("Shooter/Solve/LatencyCompOffsetX", compensatedOffset.getX());
			Logger.recordOutput("Shooter/Solve/LatencyCompOffsetY", compensatedOffset.getY());
		}

		// Precompute turret-tip tangential velocity from robot angular velocity and shooter offset
		Translation2d turretTipVel = new Translation2d();
		if (USE_TURRET_TIP_VELOCITY && robotAngularVelocityRadPerSec != 0.0) {
			Translation3d offset3 = ShooterConstants.ROBOT_TO_SHOOTER.getTranslation();
			double rx = offset3.getX();
			double ry = offset3.getY();
			// v = omega x r -> ( -omega * ry, omega * rx )
			turretTipVel = new Translation2d(-robotAngularVelocityRadPerSec * ry, robotAngularVelocityRadPerSec * rx);
			Logger.recordOutput("Shooter/Solve/TurretTipVelX", turretTipVel.getX());
			Logger.recordOutput("Shooter/Solve/TurretTipVelY", turretTipVel.getY());
		}

		Logger.recordOutput("Shooter/Solve/VelX", robotVelocity.getX());
		Logger.recordOutput("Shooter/Solve/VelY", robotVelocity.getY());

		double predictedDistance = baseDistance;
		Translation2d virtualOffset = new Translation2d();
		int iterations = 0;

		if (USE_NEWTON) {
			// Newton root-find on H(d) = virtualDistance(d) - d
			double d = predictedDistance;
			for (int i = 0; i < SOLVER_ITERATIONS; i++) {
				iterations = i + 1;
				double vd = virtualDistanceFor(d, toTarget, robotVelocity, turretTipVel, table);
				double H = vd - d;
				if (Math.abs(H) < CONVERGENCE_THRESHOLD) {
					predictedDistance = vd;
					break;
				}
				double eps = Math.max(NEWTON_EPS, d * 1e-3);
				double vdPlus = virtualDistanceFor(d + eps, toTarget, robotVelocity, turretTipVel, table);
				double vdMinus = virtualDistanceFor(Math.max(d - eps, 0.0), toTarget, robotVelocity, turretTipVel, table);
				double Hprime = ((vdPlus - vdMinus) / (2.0 * eps)) - 1.0;
				if (Math.abs(Hprime) < 1e-6) {
					// Fallback to simple recursion step
					d = vd;
				} else {
					d = d - H / Hprime;
				}
				d = Math.max(0.0, clamp(d, table.samples));
				predictedDistance = d;
			}
		} else {
			// Simple TOF recursion (document Method B)
			for (int i = 0; i < SOLVER_ITERATIONS; i++) {
				iterations = i + 1;
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

				double tof = params.timeOfFlight();
				if (USE_EFFECTIVE_TOF && DRAG_K > 0) {
					tof = effectiveTOF(tof, DRAG_K);
				}

				Translation2d drift = robotVelocity.times(tof).plus(turretTipVel.times(tof));
				virtualOffset = drift.unaryMinus();

				// Guard virtual offset
				if (virtualOffset.getNorm() > MAX_VIRTUAL_OFFSET) {
					virtualOffset = virtualOffset.times(MAX_VIRTUAL_OFFSET / virtualOffset.getNorm());
				}

				Translation2d virtualPos = toTarget.plus(virtualOffset);
				double newDistance = virtualPos.getNorm();

				Logger.recordOutput("Shooter/Solve/Iter" + i + "_Dist", predictedDistance);

				if (Math.abs(newDistance - predictedDistance) < CONVERGENCE_THRESHOLD) {
					predictedDistance = newDistance;
					break;
				}

				predictedDistance = newDistance;
			}
		}

		Logger.recordOutput("Shooter/Solve/Iterations", iterations);
		Logger.recordOutput("Shooter/Solve/FinalDist", predictedDistance);

		// Final solution
		double finalDist = clamp(predictedDistance, table.samples);
		ShotParams finalParams = interpolate(finalDist, table.samples);
		if (finalParams == null) {
			Logger.recordOutput("Shooter/Solve/Error", "Final interp failed");
			return null;
		}

		// Recompute final virtualOffset using final TOF & options for reporting
		double finalTOF = finalParams.timeOfFlight();
		if (USE_EFFECTIVE_TOF && DRAG_K > 0) finalTOF = effectiveTOF(finalTOF, DRAG_K);
		virtualOffset = robotVelocity.times(finalTOF).plus(turretTipVel.times(finalTOF)).unaryMinus();
		if (virtualOffset.getNorm() > MAX_VIRTUAL_OFFSET) virtualOffset = virtualOffset.times(MAX_VIRTUAL_OFFSET / virtualOffset.getNorm());

		// Safety check
		if (!isAngleSafe(finalParams.hoodAngle())) {
			Logger.recordOutput("Shooter/Solve/Error", "Unsafe angle");
			Logger.recordOutput("Shooter/Solve/UnsafeAngle", finalParams.hoodAngle().in(Degrees));
			return null;
		}

		Pose2d virtualTarget = new Pose2d(targetPose.getTranslation().plus(virtualOffset), targetPose.getRotation());

		double confidence = calcConfidence(finalDist, table, robotVelocity.getNorm(), iterations);

		Logger.recordOutput("Shooter/Solve/Confidence", confidence);
		Logger.recordOutput("Shooter/Solve/HoodAngle", finalParams.hoodAngle().in(Degrees));
		Logger.recordOutput("Shooter/Solve/FlywheelSpeed", table.flywheelSpeed.in(RotationsPerSecond));
		Logger.recordOutput("Shooter/Solve/TOF", finalParams.timeOfFlight());

		return new ShotSolution(finalParams.hoodAngle(), table.flywheelSpeed, virtualTarget, finalParams.timeOfFlight(), confidence, new DebugInfo(table.name, iterations, finalDist, virtualOffset));
	}

	// ========== TABLE SELECTION ==========

	private static ShotTable selectTable(double dist) {
		// Allow forcing the NEAR table for testing/tuning
		if (ShotSolverConstants.FORCE_NEAR_TABLE && !NEAR_TABLE.isEmpty()) {
			Logger.recordOutput("Shooter/Wrapper/ForcedTable", "NEAR");
			return new ShotTable(NEAR_TABLE, ShooterConstants.NEAR_FLYWHEEL_SPEED, "NEAR");
		}
		if (!NEAR_TABLE.isEmpty() && isInRange(dist, NEAR_TABLE)) return new ShotTable(NEAR_TABLE, ShooterConstants.NEAR_FLYWHEEL_SPEED, "NEAR");
		if (!FAR_TABLE.isEmpty() && isInRange(dist, FAR_TABLE)) return new ShotTable(FAR_TABLE, ShooterConstants.FAR_FLYWHEEL_SPEED, "FAR");

		// If outside ranges, pick nearest table by comparing distance to table centers
		if (!NEAR_TABLE.isEmpty() && !FAR_TABLE.isEmpty()) {
			double nearCenter = (NEAR_TABLE.get(0).distance() + NEAR_TABLE.get(NEAR_TABLE.size() - 1).distance()) / 2.0;
			double farCenter = (FAR_TABLE.get(0).distance() + FAR_TABLE.get(FAR_TABLE.size() - 1).distance()) / 2.0;
			if (Math.abs(dist - nearCenter) <= Math.abs(dist - farCenter)) return new ShotTable(NEAR_TABLE, ShooterConstants.NEAR_FLYWHEEL_SPEED, "NEAR");
			else return new ShotTable(FAR_TABLE, ShooterConstants.FAR_FLYWHEEL_SPEED, "FAR");
		}

		if (!NEAR_TABLE.isEmpty()) return new ShotTable(NEAR_TABLE, ShooterConstants.NEAR_FLYWHEEL_SPEED, "NEAR");
		if (!FAR_TABLE.isEmpty()) return new ShotTable(FAR_TABLE, ShooterConstants.FAR_FLYWHEEL_SPEED, "FAR");

		return new ShotTable(new ArrayList<>(), RotationsPerSecond.of(0), "NONE");
	}

	private static boolean isInRange(double dist, List<ShotSample> table) {
		if (table.isEmpty()) return false;
		return dist >= table.get(0).distance() - DISTANCE_EPSILON && dist <= table.get(table.size() - 1).distance() + DISTANCE_EPSILON;
	}

	// ========== INTERPOLATION / TOF HELPERS ==========

	private static ShotParams interpolate(double dist, List<ShotSample> table) {
		if (table == null || table.isEmpty()) return null;
		if (dist < table.get(0).distance() - DISTANCE_EPSILON) return null;
		if (dist > table.get(table.size() - 1).distance() + DISTANCE_EPSILON) return null;

		if (Math.abs(dist - table.get(0).distance()) < DISTANCE_EPSILON) {
			ShotSample s = table.get(0);
			return new ShotParams(s.hoodAngle(), s.timeOfFlight());
		}

		int last = table.size() - 1;
		if (Math.abs(dist - table.get(last).distance()) < DISTANCE_EPSILON) {
			ShotSample s = table.get(last);
			return new ShotParams(s.hoodAngle(), s.timeOfFlight());
		}

		int lower = binarySearchFloor(table, dist);
		int upper = Math.min(lower + 1, last);

		ShotSample lo = table.get(lower);
		ShotSample hi = table.get(upper);
		double range = hi.distance() - lo.distance();
		if (Math.abs(range) < DISTANCE_EPSILON) return new ShotParams(lo.hoodAngle(), lo.timeOfFlight());
		double t = (dist - lo.distance()) / range;
		return new ShotParams(Degrees.of(lerp(lo.hoodAngle().in(Degrees), hi.hoodAngle().in(Degrees), t)), lerp(lo.timeOfFlight(), hi.timeOfFlight(), t));
	}

	private static int binarySearchFloor(List<ShotSample> table, double dist) {
		int left = 0, right = table.size() - 1, result = 0;
		while (left <= right) {
			int mid = left + (right - left) / 2;
			if (table.get(mid).distance() <= dist) {
				result = mid;
				left = mid + 1;
			} else right = mid - 1;
		}
		return result;
	}

	// Compute virtual distance for a candidate distance d used by Newton finite-difference
	private static double virtualDistanceFor(double candidateDist, Translation2d toTarget, Translation2d robotVel, Translation2d turretTipVel, ShotTable table) {
		double clamped = clamp(candidateDist, table.samples);
		ShotParams params = interpolate(clamped, table.samples);
		if (params == null) return Double.POSITIVE_INFINITY;
		double tof = params.timeOfFlight();
		if (USE_EFFECTIVE_TOF && ShotSolverConstants.DRAG_K > 0) tof = effectiveTOF(tof, ShotSolverConstants.DRAG_K);
		Translation2d drift = robotVel.times(tof).plus(turretTipVel.times(tof));
		Translation2d virtualPos = toTarget.plus(drift.unaryMinus());
		return virtualPos.getNorm();
	}

	private static double effectiveTOF(double tau, double k) {
		if (k <= 0) return tau;
		return (1.0 - Math.exp(-k * tau)) / k;
	}

	// ========== SAFETY / CONFIDENCE / UTIL ==========

	private static boolean isAngleSafe(Angle angle) {
		double deg = angle.in(Degrees);
		return deg >= ShooterConstants.HOOD_MIN_ANGLE.in(Degrees) && deg <= ShooterConstants.HOOD_MAX_ANGLE.in(Degrees);
	}

	private static double calcConfidence(double dist, ShotTable table, double velMag, int iters) {
		if (table.samples.isEmpty()) return 0.0;
		double min = table.samples.get(0).distance();
		double max = table.samples.get(table.samples.size() - 1).distance();
		double center = (min + max) / 2.0;
		double range = Math.max(1e-6, max - min);
		double distConf = 1.0 - Math.min(1.0, Math.abs(dist - center) / (range / 2.0));
		double velConf = 1.0 - Math.min(1.0, velMag / 3.0);
		double convConf = 1.0 - (0.5 * (iters - 1) / Math.max(1, SOLVER_ITERATIONS - 1));
		double conf = 0.5 * distConf + 0.3 * velConf + 0.2 * convConf;
		Logger.recordOutput("Shooter/Confidence/Dist", distConf);
		Logger.recordOutput("Shooter/Confidence/Vel", velConf);
		Logger.recordOutput("Shooter/Confidence/Conv", convConf);
		return Math.max(0.0, Math.min(1.0, conf));
	}

	private static double clamp(double dist, List<ShotSample> table) {
		if (table.isEmpty()) return dist;
		return Math.max(table.get(0).distance(), Math.min(dist, table.get(table.size() - 1).distance()));
	}

	private static double lerp(double a, double b, double t) { return a + (b - a) * t; }

	// ========== INNER RECORDS ==========

	private static record ShotTable(List<ShotSample> samples, AngularVelocity flywheelSpeed, String name) {}
	private static record ShotParams(Angle hoodAngle, double timeOfFlight) {}

}