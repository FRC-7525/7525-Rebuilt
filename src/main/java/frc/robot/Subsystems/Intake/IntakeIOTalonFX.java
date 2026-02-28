package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Subsystems.Intake.IntakeConstants.Real;

public class IntakeIOTalonFX implements IntakeIO {

	protected final TalonFX spinMotor;
	protected final TalonFX linearMotor; // idk what to call it but the linear actuator?

	private final PositionVoltage linearRequest = new PositionVoltage(0); //talon pid cool
	private final VelocityVoltage spinRequest = new VelocityVoltage(0);

	public IntakeIOTalonFX() {
		spinMotor = new TalonFX(Real.SPIN_MOTOR_ID);
		spinMotor.setNeutralMode(NeutralModeValue.Coast);
		var spinConfig = new TalonFXConfiguration();
		spinConfig.CurrentLimits.StatorCurrentLimit = 60; // Change after testing
		spinConfig.Slot0 = SPIN_SLOT_0_CONFIGS;
		spinMotor.getConfigurator().apply(spinConfig);

		linearMotor = new TalonFX(Real.LINEAR_ACTUATOR_ID);
		linearMotor.setNeutralMode(NeutralModeValue.Brake);
		var linearConfig = new TalonFXConfiguration();
		linearConfig.Slot0 = LINEAR_SLOT_0_CONFIGS;
		//talon pid is at 1kHz, roborio at 50Hz so I think this is better for something linear?
		linearConfig.CurrentLimits.StatorCurrentLimit = 40; //change after testing
		linearConfig.Feedback.SensorToMechanismRatio = LINEAR_GEARING; //1 spin of motor = some geared extension
		linearMotor.getConfigurator().apply(linearConfig);
	}

	@Override
	public void logOutputs(IntakeIOOutputs outputs) {
		outputs.spinVelocity = spinMotor.getVelocity().getValue();
		outputs.spinAppliedVolts = spinMotor.getMotorVoltage().getValue();
		outputs.spinCurrentAmps = spinMotor.getStatorCurrent().getValue();

		outputs.linearPosition = Meters.of(linearMotor.getPosition().getValueAsDouble() * LINEAR_METERS_PER_ROTATION);
		outputs.linearVelocity = MetersPerSecond.of(linearMotor.getVelocity().getValueAsDouble() * LINEAR_METERS_PER_ROTATION);
		outputs.linearAppliedVolts = linearMotor.getMotorVoltage().getValue();
		outputs.linearCurrentAmps = linearMotor.getStatorCurrent().getValue();
	}

	@Override
	public void setSpinVelocity(AngularVelocity speed, double feedforward) {
		spinMotor.setControl(spinRequest.withVelocity(speed.in(RotationsPerSecond)).withFeedForward(feedforward));
	}

	@Override
	public void setLinearPosition(Distance setpoint) {
		linearMotor.setControl(linearRequest.withPosition(setpoint.in(Meters) / LINEAR_METERS_PER_ROTATION));
	}

	@Override
	public TalonFX getSpinMotor() {
		return spinMotor;
	}
}
