package frc.robot.Subsystems.Shooter.ShotSolver;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;
import edu.wpi.first.units.measure.Angle;

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
	
	/**
	 * Near shot table: Close-range shots (1-3 meters)
	 * Flywheel speed: NEAR_FLYWHEEL_SPEED
	 */
	public static final List<ShotSample> NEAR_SHOT_SAMPLES = List.of(
		new ShotSample(1.0, Degrees.of(87), 1.78),
		new ShotSample(1.5, Degrees.of(85), 1.78),
		new ShotSample(2.0, Degrees.of(83), 1.76),
		new ShotSample(2.5, Degrees.of(82), 1.76),
		new ShotSample(3.0, Degrees.of(79), 1.74),
		new ShotSample(3.5, Degrees.of(77), 1.72),
		new ShotSample(4.0, Degrees.of(75), 1.72),
		new ShotSample(4.5, Degrees.of(73), 1.68),
		new ShotSample(5.0, Degrees.of(71), 1.66)
	);
	
	/**
	 * Medium shot table: Mid-range shots (3-5 meters)
	 * Flywheel speed: MEDIUM_FLYWHEEL_SPEED
	 */
	public static final List<ShotSample> MEDIUM_SHOT_SAMPLES = List.of(
		new ShotSample(3.0, Degrees.of(35.0), 0.60),
		new ShotSample(3.5, Degrees.of(32.0), 0.70),
		new ShotSample(4.0, Degrees.of(30.0), 0.80),
		new ShotSample(4.5, Degrees.of(28.0), 0.90),
		new ShotSample(5.0, Degrees.of(26.0), 1.00)
	);
	
	/**
	 * Far shot table: Long-range shots (5-8 meters)
	 * Flywheel speed: FAR_FLYWHEEL_SPEED
	 */
	public static final List<ShotSample> FAR_SHOT_SAMPLES = List.of(
		new ShotSample(5.0, Degrees.of(30.0), 0.90),
		new ShotSample(5.5, Degrees.of(28.0), 1.00),
		new ShotSample(6.0, Degrees.of(26.0), 1.10),
		new ShotSample(6.5, Degrees.of(24.0), 1.20),
		new ShotSample(7.0, Degrees.of(22.0), 1.30),
		new ShotSample(7.5, Degrees.of(20.0), 1.40),
		new ShotSample(8.0, Degrees.of(18.0), 1.50)
	);
}
