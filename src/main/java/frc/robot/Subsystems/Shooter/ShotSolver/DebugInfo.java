package frc.robot.Subsystems.Shooter.ShotSolver;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Debug information for shot telemetry and troubleshooting.
 */
public class DebugInfo {
	private final String tableUsed;
	private final int iterations;
	private final double finalDistance;
	private final Translation2d virtualOffset;
	
	public DebugInfo(
			String tableUsed,
			int iterations,
			double finalDistance,
			Translation2d virtualOffset) {
		this.tableUsed = tableUsed;
		this.iterations = iterations;
		this.finalDistance = finalDistance;
		this.virtualOffset = virtualOffset;
	}
	
	/** Which shot table was used (NEAR/MEDIUM/FAR) */
	public String getTableUsed() {
		return tableUsed;
	}
	
	/** Number of iterations to converge */
	public int getIterations() {
		return iterations;
	}
	
	/** Final predicted distance after convergence */
	public double getFinalDistance() {
		return finalDistance;
	}
	
	/** Virtual target offset from actual target (accounts for robot motion) */
	public Translation2d getVirtualOffset() {
		return virtualOffset;
	}
	
	@Override
	public String toString() {
		return String.format(
			"Debug[table=%s, iters=%d, dist=%.2fm, offset=(%.2f, %.2f)]",
			tableUsed,
			iterations,
			finalDistance,
			virtualOffset.getX(),
			virtualOffset.getY()
		);
	}
}