package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.Intake.IntakeConstants.Real;

public class IntakeIOTalonFX implements IntakeIO {

	private final TalonFX spinMotor;
	private final TalonFX linearMotor; // idk what to call it but the linear actuator?

	private final PositionVoltage linearRequest = new PositionVoltage(0); //talon pid cool

	public IntakeIOTalonFX() {
		spinMotor = new TalonFX(Real.SPIN_MOTOR_ID);
		spinMotor.setNeutralMode(NeutralModeValue.Coast);
		var spinConfig = new TalonFXConfiguration();
		spinConfig.CurrentLimits.StatorCurrentLimit = 60; // Change after testing
		spinMotor.getConfigurator().apply(spinConfig);

		linearMotor = new TalonFX(Real.LINEAR_ACTUATOR_ID);
		linearMotor.setNeutralMode(NeutralModeValue.Brake);
		var linearConfig = new TalonFXConfiguration();
		linearConfig.Slot0 = LINEAR_SLOT_0_CONFIGS; 
        //talon pid is at 1kHz, roborio at 50Hz so I think this is better for something linear?
		linearConfig.CurrentLimits.StatorCurrentLimit = 40; //change after testing
		linearMotor.getConfigurator().apply(linearConfig);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.spinVelocityRPS = spinMotor.getVelocity().getValue().in(RotationsPerSecond);
		inputs.spinAppliedVolts = spinMotor.getMotorVoltage().getValueAsDouble();
		inputs.spinCurrentAmps = spinMotor.getStatorCurrent().getValueAsDouble();

		inputs.linearPositionRotations = linearMotor.getPosition().getValueAsDouble();
		inputs.linearVelocityRPS = linearMotor.getVelocity().getValueAsDouble();
		inputs.linearAppliedVolts = linearMotor.getMotorVoltage().getValueAsDouble();
		inputs.linearCurrentAmps = linearMotor.getStatorCurrent().getValueAsDouble();
	}

	@Override
	public void setSpinVoltage(double volts) {
		spinMotor.setVoltage(volts);
	}

	@Override
	public void setLinearPosition(double rotations) {
		linearMotor.setControl(linearRequest.withPosition(rotations));
	}

	@Override
	public TalonFX getSpinMotor() {
		return spinMotor;
	}
}