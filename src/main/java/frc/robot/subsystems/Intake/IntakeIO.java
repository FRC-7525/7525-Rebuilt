package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

public interface IntakeIO {
	public static class IntakeIOInputs {

		public double spinVelocityRPS;
		public double spinAppliedVolts;
		public double spinCurrentAmps;

		public double linearPositionMeters;
		public double linearVelocityMetersPerSec;
		public double linearAppliedVolts;
		public double linearCurrentAmps;
	}

	public void updateInputs(IntakeIOInputs inputs);

	public void setSpinVelocity(double rps, double feedforward);

	public void setLinearPosition(double meters);

	public TalonFX getSpinMotor();
}