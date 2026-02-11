package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;

public class ClimberIOReal implements ClimberIO {

	protected TalonFX leftMotor;
	protected TalonFX rightMotor;
	protected Angle positionSetpoint;
	protected PIDController pid;

	public ClimberIOReal() {
		leftMotor = new TalonFX(ClimberConstants.LEFT_CLIMBER_MOTOR_ID);
		rightMotor = new TalonFX(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID);
		rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned));

		pid = ClimberConstants.CLIMB_PID.get();
		positionSetpoint = ClimberConstants.IDLE_SETPOINT;
	}

	@Override
	public void logOutputs(ClimberIOOutputs outputs) {
		outputs.leftPosition = leftMotor.getPosition().getValue();
		outputs.rightPosition = rightMotor.getPosition().getValue();
		outputs.setpoint = positionSetpoint;

		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/LeftPositionRot", outputs.leftPosition.in(Rotations));
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/RightPositionRot", outputs.rightPosition.in(Rotations));
		Logger.recordOutput(ClimberConstants.SUBSYSTEM_NAME + "/SetpointRot", outputs.setpoint.in(Rotations));
	}

	@Override
	public void setPosition(Angle position) {
		this.positionSetpoint = position;
		leftMotor.set(pid.calculate(leftMotor.getPosition().getValue().in(Rotations), positionSetpoint.in(Rotations)));
	}

	@Override
	public boolean atPositionSetpoint() {
		double currentRot = leftMotor.getPosition().getValue().in(Rotations);
		return Math.abs(currentRot - positionSetpoint.in(Rotations)) < ClimberConstants.CLIMB_POSITION_TOLERANCE.in(Rotations);
	}
}
