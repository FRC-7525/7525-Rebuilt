package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
	@AutoLog
	class IntakeIOInputs {

		public double spinVelocityRPS;
		public double spinAppliedVolts;
		public double spinCurrentAmps;

		public double linearPositionRotations;
		public double linearVelocityRPS;
		public double linearAppliedVolts;
		public double linearCurrentAmps;
	}

	public void updateInputs(IntakeIOInputs inputs);

	public void setSpinVoltage(double volts);

	public void setLinearPosition(double rotations);

	public TalonFX getSpinMotor();
}