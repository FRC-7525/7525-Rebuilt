package frc.robot.Subsytems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.team7525.subsystem.SubsystemStates;

import edu.wpi.first.units.measure.Angle;
import static frc.robot.Subsytems.Shooter.ShooterConstants.*;
import edu.wpi.first.units.measure.AngularVelocity;

public enum ShooterStates implements SubsystemStates {
    OFF("IDLE", () -> Degrees.of(0), () -> RotationsPerSecond.of(0)),
    REVERSE("REVERSE", () -> Degrees.of(0), () -> RotationsPerSecond.of(-100)), // TODO: get good value
    // Use placeholder Pose2d and zero velocity for now; replace with real robot pose/velocity when available.
    SHOOT_HUB("SHOOT HUB",
    () -> ShooterMath.solveHubShot(new Pose2d(0.0, 0.0, new Rotation2d()), new Translation2d(0.0, 0.0))
        .map(sol -> sol.hoodAngle()).orElse(FIXED_SHOT_ANGLE),
    () -> ShooterMath.solveHubShot(new Pose2d(0.0, 0.0, new Rotation2d()), new Translation2d(0.0, 0.0))
        .map(sol -> sol.flywheelSpeed()).orElse(FIXED_SHOT_SPEED)),

    SHOOT_ALLIANCE("SHOOT ALLIANCE",
    () -> ShooterMath.solveAllianceShot(new Pose2d(0.0, 0.0, new Rotation2d()), new Translation2d(0.0, 0.0))
        .map(sol -> sol.hoodAngle()).orElse(FIXED_SHOT_ANGLE),
    () -> ShooterMath.solveAllianceShot(new Pose2d(0.0, 0.0, new Rotation2d()), new Translation2d(0.0, 0.0))
        .map(sol -> sol.flywheelSpeed()).orElse(FIXED_SHOT_SPEED)),
    SHOOT_FIXED("SHOOT FIXED", () -> FIXED_SHOT_ANGLE, () -> FIXED_SHOT_SPEED);

    private String stateString;
    private Supplier<Angle> hoodAngleSupplier;
    private Supplier<AngularVelocity> wheelVelocitySupplier;

    private ShooterStates(String stateString, Supplier<Angle> hoodAngleSupplier, Supplier<AngularVelocity> wheelVelocitySupplier) {
        this.stateString = stateString;
        this.hoodAngleSupplier = hoodAngleSupplier;
        this.wheelVelocitySupplier = wheelVelocitySupplier;
    }

    public String getStateString() {
        return stateString;
    }
    
    public Angle getHoodAngle() {
        return hoodAngleSupplier.get();
    
    }

    public AngularVelocity getWheelVelocity() {
        return wheelVelocitySupplier.get();
    }
}
