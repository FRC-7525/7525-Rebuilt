package frc.robot.Subsystems.Shooter.ShotSolver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * Complete solution for executing a shot.
 * Contains all parameters needed: angle, speed, aim point, timing, and confidence.
 */
public class ShotSolution {
	private final Angle hoodAngle;
	private final AngularVelocity flywheelSpeed;
	private final Pose2d virtualTarget;
	private final double timeOfFlight;
	private final double confidence;
	private final DebugInfo debugInfo;
	
	public ShotSolution(
			Angle hoodAngle,
			AngularVelocity flywheelSpeed,
			Pose2d virtualTarget,
			double timeOfFlight,
			double confidence,
			DebugInfo debugInfo) {
		this.hoodAngle = hoodAngle;
		this.flywheelSpeed = flywheelSpeed;
		this.virtualTarget = virtualTarget;
		this.timeOfFlight = timeOfFlight;
		this.confidence = confidence;
		this.debugInfo = debugInfo;
	}
	
	// ========== GETTERS ==========
	
	/** Hood/pivot angle to set */
	public Angle getHoodAngle() {
		return hoodAngle;
	}
	
	/** Flywheel speed to set */
	public AngularVelocity getFlywheelSpeed() {
		return flywheelSpeed;
	}
	
	/** Where to aim (turret/robot rotation) - accounts for robot motion */
	public Pose2d getVirtualTarget() {
		return virtualTarget;
	}
	
	/** Predicted time for game piece to reach target (seconds) */
	public double getTimeOfFlight() {
		return timeOfFlight;
	}
	
	/** Shot confidence from 0.0 (low) to 1.0 (high) */
	public double getConfidence() {
		return confidence;
	}
	
	/** Debug/telemetry information */
	public DebugInfo getDebugInfo() {
		return debugInfo;
	}
	
	// ========== UTILITY ==========
	
	/** Check if shot has high confidence (>0.8) */
	public boolean isHighConfidence() {
		return confidence > 0.8;
	}
	
	/** Check if shot has acceptable confidence (>0.5) */
	public boolean isAcceptable() {
		return confidence > 0.5;
	}
	
	@Override
	public String toString() {
		return String.format(
			"ShotSolution[hood=%.1f°, wheels=%.0f RPS, target=(%.2f, %.2f, %.1f°), TOF=%.2fs, conf=%.2f]",
			hoodAngle.in(Degrees),
			flywheelSpeed.in(RotationsPerSecond),
			virtualTarget.getX(),
			virtualTarget.getY(),
			virtualTarget.getRotation().getDegrees(),
			timeOfFlight,
			confidence
		);
	}
}