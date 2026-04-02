package frc.robot.Subsystems.Climber;

import static frc.robot.Subsystems.Climber.ClimberConstants.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import org.littletonrobotics.junction.Logger;

public class ClimberIOReal implements ClimberIO {

	protected TalonFX motor;
	protected Angle angularPositionSetpoint;
	protected PIDController pid;

	public ClimberIOReal() {
		motor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);

		pid = ClimberConstants.CLIMB_PID.get();
		angularPositionSetpoint = ClimberConstants.IDLE_SETPOINT;
	}

	@Override
	public void logOutputs(ClimberIOOutputs outputs) {
		outputs.angularPos = motor.getPosition().getValue();
		outputs.angularSetpoint = angularPositionSetpoint;
	}

	@Override
	public void setPosition(Angle position) {
		this.angularPositionSetpoint = position;
		motor.set(pid.calculate(motor.getPosition().getValue().in(Rotations), angularPositionSetpoint.in(Rotations)));
	}

	@Override
	public boolean atPositionSetpoint() {
		double currentRot = motor.getPosition().getValue().in(Rotations);
		return Math.abs(currentRot - angularPositionSetpoint.in(Rotations)) < ClimberConstants.CLIMB_POSITION_TOLERANCE.in(Rotations);
	}
}
