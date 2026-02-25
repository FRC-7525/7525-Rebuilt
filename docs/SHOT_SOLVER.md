## Shot Solver (SOTF) — Overview and Tuning Guide

This document explains the shoot-on-the-move (SOTF) solver added to the project, where to find the implementation, runtime toggles, and how to tune it for your robot.

## High-level approach

- We use a "virtual target" recursion approach: predict where the robot (and turret tip) will be when the ball reaches the hub, then use measured-shot lookup tables (LUTs) to pick hood angle and time-of-flight (TOF).
 - Two solver styles are supported:
   - Recursion (Method B): iterate TOF -> drift -> virtual target -> distance until convergence.
   - Newton (optional): root-find using a finite-difference derivative for faster convergence (can be enabled via toggle).
 - The LUTs are reduced to two tables: NEAR and FAR. Each has distinct measured values for REAL (hardware) and SIM; the runtime selection is automatic based on `GlobalConstants.ROBOT_MODE`.
- Latency compensation, drag/effective TOF, and turret-tip tangential velocity are included.

## Where to find the code

- Wrapper API (stable surface): `src/main/java/frc/robot/Subsystems/Shooter/ShotSolver/ShooterMath.java`
  - Small wrapper that delegates to the experimental solver and records compact logging of solver flags.
- Full solver implementation: `src/main/java/frc/robot/Subsystems/Shooter/ShotSolver/ShooterMath.java`
  - Contains full implementation of recursion, Newton option, latency projection, turret-tip velocity, table selection, interpolation, safety checks, and confidence heuristics. This is the single authoritative solver implementation.
- Solver-specific toggles/constants: `src/main/java/frc/robot/Subsystems/Shooter/ShotSolver/ShotSolverConstants.java`
  - Toggle `USE_NEWTON`, `USE_EFFECTIVE_TOF`, `USE_LATENCY_COMPENSATION`, `USE_TURRET_TIP_VELOCITY` and tune numeric constants (`SENSOR_LATENCY_SECONDS`, `DRAG_K`, `SOLVER_ITERATIONS`, etc.)
- LUTs / sample data: `src/main/java/frc/robot/Subsystems/Shooter/ShotSolver/ShotTables.java`
  - Contains the measured samples used for interpolation (NEAR, MEDIUM, FAR tables).
- Output / debug types: `ShotSolution` and `DebugInfo` (same package).

## Logging

- High-level wrapper logs present in `ShooterMath.java`:
  - `Shooter/Wrapper/UseNewton` (boolean)
  - `Shooter/Wrapper/UseEffectiveTOF` (boolean)
  - `Shooter/Wrapper/UseLatencyComp` (boolean)
  - `Shooter/Wrapper/SensorLatencySeconds` (double)
-- Detailed per-solve logs are in `ShooterMath.java` using `Logger.recordOutput(...)` for
  - selected table, iteration counts, per-iteration distances, TOF, final hood angle, flywheel setpoint, and confidence.
- Use the logging system in simulation/field to capture these values for tuning and validation.

## Tuning guide (practical steps)

1. Verify table data
   - Ensure `ShotTables` contains accurate measured samples (distance, hood angle, measured TOF).
   - If available, add per-sample exit speed & variance (future improvement) to improve confidence scoring.

2. Latency
   - `SENSOR_LATENCY_SECONDS` projects robot position forward before computing the shot.
   - Start with the measured camera+processing latency (e.g., 0.08 - 0.15s). Observe the recorded `Shooter/Solve/LatencyCompOffsetX/Y` and `Shooter/Wrapper/SensorLatencySeconds` in logs.
   - Tune until mean residual (measured TOF vs predicted TOF) is minimized.

3. Drag / Effective TOF (`DRAG_K`)
   - `DRAG_K` is used to compute an effective TOF: alpha(tau) = (1 - exp(-k * tau)) / k
   - If shots consistently under/over-compensate when the robot is moving fast, increase/decrease `DRAG_K`.
   - Use recorded `Shooter/Solve/TOF` vs actual TOF measured from sim/field to iterate.

4. Newton vs Recursion
   - `USE_NEWTON` toggles the Newton-based solver. Newton converges faster but can be unstable if LUT interpolation is noisy.
   - Start with recursion (`USE_NEWTON = false`). Once tables are clean and consistent, enable Newton and watch `Shooter/Solve/Iterations` and `Shooter/Solve/Success`.

5. Turret-tip tangential velocity
   - If the shooter is offset from robot center, enable `USE_TURRET_TIP_VELOCITY` to include tangential velocity from robot angular velocity (omega x r).
   - Confirm that `ShooterConstants.ROBOT_TO_SHOOTER` reflects the real transform.

6. Confidence and safety
   - The solver computes a `confidence` metric (`Shooter/Solve/Confidence`) combining distance-from-center, robot speed, and convergence.
   - Use this to gate automatic firing or to require operator confirmation for low-confidence shots.

## Quick checks to run in sim

- In `ShooterIOSim` you can spawn fuel shots and check the `T_OF_F` printed to SmartDashboard.
- Compare the logged `Shooter/Solve/TOF` to the actual measured TOF to compute residuals.
- Use multiple robot velocities and rotations to validate that latency compensation and turret-tip velocity behave correctly.

## Next steps / improvements

-- Add per-sample exit speed & variance to `ShotTables` and use those values in `ShooterMath` to improve interpolation and confidence scoring.
- Add an automated test harness that runs a grid of robot velocities and positions and reports convergence, residuals, and failures.
- Consider recording historical shot errors in a CSV while tuning and plotting them externally.

If you want, I can now:
- Run a small sim-run that spawns a set of test shots and collects log output for tuning, or
- Add per-sample exit speed to `ShotTables` and wire it through the solver.

