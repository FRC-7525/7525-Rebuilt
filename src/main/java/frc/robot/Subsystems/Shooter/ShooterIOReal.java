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
	}

	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		wheelSetpoint = velocity;
		leftMotor.set(wheelPID.calculate(leftMotor.getVelocity().getValue().in(RotationsPerSecond), wheelSetpoint.in(RotationsPerSecond)) + wheelFeedforward.calculate(wheelSetpoint.in(RotationsPerSecond)));
	}

	@Override
	public void setHoodAngle(Angle angle) {
		hoodSetpoint = angle;
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
