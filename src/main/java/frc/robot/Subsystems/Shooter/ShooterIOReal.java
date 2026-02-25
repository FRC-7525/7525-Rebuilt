package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;

public class ShooterIOReal implements ShooterIO {

	protected TalonFX leftMotor;
	protected TalonFX rightMotor;
	protected TalonFX hoodMotor;
	protected Angle hoodSetpoint;
	protected AngularVelocity wheelSetpoint;
	protected PIDController hoodPID;
	protected PIDController wheelPID;
	protected SimpleMotorFeedforward wheelFeedforward;

	public ShooterIOReal() {
		wheelSetpoint = RotationsPerSecond.zero();
		hoodSetpoint = Degrees.zero();
		leftMotor = new TalonFX(LEFT_SHOOTER_MOTOR_ID);
		rightMotor = new TalonFX(RIGHT_SHOOTER_MOTOR_ID);
		hoodMotor = new TalonFX(HOOD_MOTOR_ID);
		rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned)); // Might need to be inverted
		hoodPID = HOOD_PID.get();
		wheelPID = WHEEL_PID.get();
		wheelFeedforward = WHEEL_FEEDFORWARD.get();
	}

	@Override
	public void logOutputs(ShooterIOOutputs outputs) {
		outputs.leftWheelVelocity = leftMotor.getVelocity().getValue();
		outputs.rightWheelVelocity = rightMotor.getVelocity().getValue();
		outputs.wheelSetpoint = wheelSetpoint;
		outputs.hoodAngle = hoodMotor.getPosition().getValue();
		outputs.hoodSetpoint = hoodSetpoint;
		// TODO: Remove before comp
		if (GlobalConstants.TESTING) {
			SmartDashboard.putData("Wheel PID", wheelPID);
			SmartDashboard.putData("Hood PID", hoodPID);
			wheelFeedforward.setKs(SmartDashboard.getNumber("Wheel Feedforward kS", wheelFeedforward.getKs()));
			wheelFeedforward.setKv(SmartDashboard.getNumber("Wheel Feedforward kV", wheelFeedforward.getKv()));
			SmartDashboard.putNumber("Wheel Feedforward kS", wheelFeedforward.getKs());
			SmartDashboard.putNumber("Wheel Feedforward kV", wheelFeedforward.getKv());
		}
	}

	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		wheelSetpoint = velocity;
		if (GlobalConstants.TESTING) {
			wheelSetpoint = RotationsPerSecond.of(SmartDashboard.getNumber("Wheel Velocity Setpoint", wheelSetpoint.in(RotationsPerSecond)));
			SmartDashboard.putNumber("Wheel Velocity Setpoint", wheelSetpoint.in(RotationsPerSecond));
			SmartDashboard.putNumber("Wheel Velocity", leftMotor.getVelocity().getValue().in(RotationsPerSecond));

		}
		leftMotor.set(wheelPID.calculate(leftMotor.getVelocity().getValue().in(RotationsPerSecond), wheelSetpoint.in(RotationsPerSecond)) + wheelFeedforward.calculate(wheelSetpoint.in(RotationsPerSecond)));
	}

	@Override
	public void setHoodAngle(Angle angle) {
		hoodSetpoint = angle;
		if (GlobalConstants.TESTING) {
			hoodSetpoint = Degrees.of(SmartDashboard.getNumber("Hood Angle Setpoint", hoodSetpoint.in(Degrees)));
			SmartDashboard.putNumber("Hood Angle Setpoint", hoodSetpoint.in(Degrees));
			SmartDashboard.putNumber("Hood Angle", hoodMotor.getPosition().getValue().in(Degrees));
		}
		hoodMotor.set(hoodPID.calculate(hoodMotor.getPosition().getValue().in(Degrees), hoodSetpoint.in(Degrees)));
	}

	@Override
	public boolean atWheelVelocitySetpoint() {
		return (Math.abs(leftMotor.getVelocity().getValue().in(RotationsPerSecond) - wheelSetpoint.in(RotationsPerSecond)) < WHEEL_VELOCITY_TOLERANCE);
	}

	@Override
	public boolean atHoodAngleSetpoint() {
		return (Math.abs(hoodMotor.getPosition().getValue().in(Degrees) - hoodSetpoint.in(Degrees)) < HOOD_ANGLE_TOLERANCE_DEGREES);
	}
}
