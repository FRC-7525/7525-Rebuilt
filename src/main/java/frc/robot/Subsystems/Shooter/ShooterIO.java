package frc.robot.Subsystems.Shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
	class ShooterIOOutputs {

		public AngularVelocity leftWheelVelocity;
		public AngularVelocity rightWheelVelocity;
		public AngularVelocity wheelSetpoint;
		public Angle hoodAngle;
		public Angle hoodSetpoint;
	}

	public abstract void logOutputs(ShooterIOOutputs outputs);

	public abstract void setWheelVelocity(AngularVelocity velocity);

	public abstract void setHoodAngle(Angle angle);

	public abstract boolean atWheelVelocitySetpoint();

	public abstract boolean atHoodAngleSetpoint();
}
