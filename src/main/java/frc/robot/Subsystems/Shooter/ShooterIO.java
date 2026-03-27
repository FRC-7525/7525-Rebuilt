package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.List;

public interface ShooterIO {
	class ShooterIOOutputs {

		public AngularVelocity leftWheelVelocity;
		public AngularVelocity rightWheelVelocity;
		public AngularVelocity wheelSetpoint;
		public Angle hoodAngle;
		public Angle hoodSetpoint;
		public double leftWheelCurrent;
		public double rightWheelCurrent;
		public double hoodCurrent;
	}

	public abstract void logOutputs(ShooterIOOutputs outputs);

	public abstract void setWheelVelocity(AngularVelocity velocity);

	public abstract void setHoodAngle(Angle angle);

	public abstract boolean atWheelVelocitySetpoint();

	public abstract boolean atHoodAngleSetpoint();

	public abstract boolean zeroHoodMotor();

	public abstract List<TalonFX> getShooterMotors();

	public abstract TalonFX getHoodMotor();

	public abstract AngularVelocity getWheelVelocity();
}
