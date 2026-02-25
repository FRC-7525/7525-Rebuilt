package frc.robot.Subsystems.Shooter.ShotSolver;

/**
 * Constants specific to the shot solver (separate from general shooter constants).
 */
public final class ShotSolverConstants {

    // Algorithm selection
    public static final boolean USE_NEWTON = false; // If true use Newton's method, else recursion
    public static final boolean USE_EFFECTIVE_TOF = true; // Use effective TOF to model drag
    public static final boolean USE_LATENCY_COMPENSATION = false; // Compensate for sensing latency
    public static final boolean USE_TURRET_TIP_VELOCITY = true; // Add tangential velocity due to robot rotation

    // Numerical / solver
    public static final int SOLVER_ITERATIONS = 10;
    public static final double CONVERGENCE_THRESHOLD = 0.001; // meters
    public static final double DISTANCE_EPSILON = 0.001; // meters
    public static final double NEWTON_EPS = 0.01; // finite diff for derivative (meters)

    // Latency (seconds) - project position forward by this much before solving
    public static final double SENSOR_LATENCY_SECONDS = 0.0;

    // Force use of the NEAR table for solver table selection (useful for testing/tuning)
    // Not final so it can be toggled at runtime for testing without producing dead-code at compile time.
    public static final boolean FORCE_NEAR_TABLE = true;

    // Drag constant for effective TOF alpha(t) = (1 - e^{-k t}) / k
    // Small k (0.5) gives close to linear; tune on robot. Set to 0 to disable effect.
    public static final double DRAG_K = 0.6;

    // Safety / guards
    public static final double MAX_VIRTUAL_OFFSET = 10; // meters; prevent huge aim offsets

    private ShotSolverConstants() {}
}
