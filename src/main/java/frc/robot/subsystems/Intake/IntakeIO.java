package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
	public static class IntakeIOOutputs {
		public AngularVelocity spinVelocity = RotationsPerSecond.of(0);
		public AngularVelocity spinSetpoint = RotationsPerSecond.of(0);
		public Voltage spinAppliedVolts = Volts.of(0);
		public Current spinCurrentAmps = Amps.of(0);

		public Distance linearPosition = Meters.of(0);
		public Distance linearSetpoint = Meters.of(0);
		public LinearVelocity linearVelocity = MetersPerSecond.of(0);
		public Voltage linearAppliedVolts = Volts.of(0);
		public Current linearCurrentAmps = Amps.of(0);
	}

	public void logOutputs(IntakeIOOutputs outputs);

	public void setSpinVelocity(AngularVelocity speed, double feedforward);

	public void setLinearPosition(Distance setpoint);

	public TalonFX getSpinMotor();
}
