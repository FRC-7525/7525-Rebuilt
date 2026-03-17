package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Subsystems.Drive.Drive;
import java.util.function.Supplier;
import org.team7525.subsystem.SubsystemStates; 

public enum ShooterStates implements SubsystemStates {
	IDLE("IDLE", () -> Degrees.of(0), () -> RotationsPerSecond.of(0)),
	ZEROING("ZEROING", () -> Degrees.of(0), () -> RotationsPerSecond.of(0)),
	SHOOT_HUB("SHOOT HUB", () -> ShooterMath.solveHubShot(Drive.getInstance().getPose()), () -> FIXED_SHOT_SPEED),
	SHOOT_ALLIANCE("SHOOT ALLIANCE", () -> ALLIANCE_SHOT_ANGLE, () -> ALLIANCE_SHOT_SPEED),
	SHOOT_FIXED("SHOOT FIXED", () -> FIXED_SHOT_ANGLE, () -> FIXED_SHOT_SPEED),
	STANDBY("STANDBY", () -> STANDBY_ANGLE, () -> STANDBY_SPEED);

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
