package frc.robot.Subsystems.Shooter.ShotSolver;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;
import frc.robot.GlobalConstants;
import edu.wpi.first.units.measure.Angle;

/**
 * Shot lookup tables. Reduced to two tables (NEAR and FAR). Each table has separate
 * measured values for REAL and SIM; the runtime `NEAR_SHOT_SAMPLES` and `FAR_SHOT_SAMPLES`
 * are selected automatically based on `GlobalConstants.ROBOT_MODE`.
 */
public class ShotTables {
    /**
	 * A single measured shot sample.
	 * Since flywheel speed is fixed per table, we only need distance, angle, and TOF.
	 * 
	 * @param distance Distance from robot to target (meters)
	 * @param hoodAngle Hood/pivot angle for this shot (degrees)
	 * @param timeOfFlight Time for game piece to reach target (seconds)
	 */
	public record ShotSample(double distance, Angle hoodAngle, double timeOfFlight) {}
	
	// ========== SHOT TABLES ==========
	// Replace these with real measured data from your robot!

	// Real (hardware) measured samples
	public static final List<ShotSample> NEAR_REAL_SHOT_SAMPLES = List.of(
		new ShotSample(1.0, Degrees.of(87), 1.78),
		new ShotSample(1.5, Degrees.of(85), 1.78),
		new ShotSample(2.0, Degrees.of(83), 1.76),
		new ShotSample(2.5, Degrees.of(82), 1.76),
		new ShotSample(3.0, Degrees.of(79), 1.74),
		new ShotSample(3.5, Degrees.of(77), 1.72),
		new ShotSample(4.0, Degrees.of(75), 1.72),
		new ShotSample(4.5, Degrees.of(73), 1.69),
		new ShotSample(5.0, Degrees.of(71), 1.66),
		new ShotSample(5.5, Degrees.of(69), 1.64),
		new ShotSample(6.0, Degrees.of(67), 1.62),
		new ShotSample(6.5, Degrees.of(63), 1.54),
		new ShotSample(7.0, Degrees.of(60), 1.50),
		new ShotSample(7.5, Degrees.of(55), 1.38),
		new ShotSample(8.0, Degrees.of(50), 1.30)
	);

	public static final List<ShotSample> FAR_REAL_SHOT_SAMPLES = List.of(
		new ShotSample(5.0, Degrees.of(30.0), 0.90),
		new ShotSample(5.5, Degrees.of(28.0), 1.00),
		new ShotSample(6.0, Degrees.of(26.0), 1.10),
		new ShotSample(6.5, Degrees.of(24.0), 1.20),
		new ShotSample(7.0, Degrees.of(22.0), 1.30)
	);

	// Simulation-friendly samples (coarser or tuned for sim physics)
	public static final List<ShotSample> NEAR_SIM_SHOT_SAMPLES = List.of(
		new ShotSample(1.0, Degrees.of(87), 1.78),
		new ShotSample(1.5, Degrees.of(85), 1.78),
		new ShotSample(2.0, Degrees.of(83), 1.76),
		new ShotSample(2.5, Degrees.of(82), 1.76),
		new ShotSample(3.0, Degrees.of(79), 1.74),
		new ShotSample(3.5, Degrees.of(77), 1.72),
		new ShotSample(4.0, Degrees.of(75), 1.72),
		new ShotSample(4.5, Degrees.of(73), 1.69),
		new ShotSample(5.0, Degrees.of(71), 1.66),
		new ShotSample(5.5, Degrees.of(69), 1.64),
		new ShotSample(6.0, Degrees.of(67), 1.62),
		new ShotSample(6.5, Degrees.of(63), 1.54),
		new ShotSample(7.0, Degrees.of(60), 1.50),
		new ShotSample(7.5, Degrees.of(55), 1.38),
		new ShotSample(8.0, Degrees.of(50), 1.30)
	);

	public static final List<ShotSample> FAR_SIM_SHOT_SAMPLES = List.of(
		new ShotSample(5.0, Degrees.of(30.0), 0.85),
		new ShotSample(6.0, Degrees.of(26.0), 0.95),
		new ShotSample(7.0, Degrees.of(22.0), 1.05),
		new ShotSample(8.0, Degrees.of(18.0), 1.15)
	);

	// Public aliases used by the solver; selected at class-init based on ROBOT_MODE
	public static final List<ShotSample> NEAR_SHOT_SAMPLES;
	public static final List<ShotSample> FAR_SHOT_SAMPLES;

	static {
		if (GlobalConstants.ROBOT_MODE == GlobalConstants.RobotMode.SIM) {
			NEAR_SHOT_SAMPLES = NEAR_SIM_SHOT_SAMPLES;
			FAR_SHOT_SAMPLES = FAR_SIM_SHOT_SAMPLES;
		} else {
			NEAR_SHOT_SAMPLES = NEAR_REAL_SHOT_SAMPLES;
			FAR_SHOT_SAMPLES = FAR_REAL_SHOT_SAMPLES;
		}
	}
}
