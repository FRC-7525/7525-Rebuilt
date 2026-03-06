package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
	public static class IntakeIOOutputs {

		public double spinVelocity = 0;
		public double spinSetpoint = 0;
		public Voltage spinAppliedVolts = Volts.of(0);
		public Current spinCurrentAmps = Amps.of(0);
		public Angle angularPosition = Degrees.of(0);
		public Angle angularSetpoint = Degrees.of(0);
	}

	public void logOutputs(IntakeIOOutputs outputs);

	public void setSpinVelocity(double speed);

	public void setAngularPosition(Angle setpoint);

	public TalonFX getSpinMotor();
}
