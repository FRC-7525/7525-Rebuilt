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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {

	protected TalonFX leftMotor;
	protected TalonFX rightMotor;
	protected TalonFX hoodMotor;
	protected Angle hoodSetpoint;
	protected AngularVelocity wheelSetpoint;
	protected PIDController hoodPID;
	protected PIDController wheelPID;
	protected SimpleMotorFeedforward wheelFeedforward;
	protected DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);

	public ShooterIOReal() {
		wheelSetpoint = RotationsPerSecond.zero();
		hoodSetpoint = Degrees.zero();
		leftMotor = new TalonFX(LEFT_SHOOTER_MOTOR_ID);
		rightMotor = new TalonFX(RIGHT_SHOOTER_MOTOR_ID);
		hoodMotor = new TalonFX(HOOD_MOTOR_ID);
		rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed)); // Might need to be inverted
		hoodPID = HOOD_PID.get();
		wheelPID = WHEEL_PID.get();
		wheelFeedforward = WHEEL_FEEDFORWARD.get();
		hoodMotor.setPosition(0);
	}

	@Override
	public void logOutputs(ShooterIOOutputs outputs) {
		outputs.leftWheelVelocity = leftMotor.getVelocity().getValue();
		outputs.rightWheelVelocity = rightMotor.getVelocity().getValue();
		outputs.wheelSetpoint = wheelSetpoint;
		outputs.hoodAngle = hoodMotor.getPosition().getValue().div(-HOOD_GEARING);
		outputs.hoodSetpoint = hoodSetpoint;
		SmartDashboard.putData("ShooterPID", wheelPID);
		wheelFeedforward.setKv(SmartDashboard.getNumber("ShooterFeedforwardKv", wheelFeedforward.getKv()));
		SmartDashboard.putNumber("ShooterFeedforwardKs", wheelFeedforward.getKs());
		wheelFeedforward.setKs(SmartDashboard.getNumber("ShooterFeedforwardKvs", wheelFeedforward.getKs()));
		Logger.recordOutput("hoodCurrent", hoodMotor.getStatorCurrent().getValue());
		SmartDashboard.putData(hoodPID);
		SmartDashboard.putNumber("CURRENT L", leftMotor.getStatorCurrent().getValueAsDouble());
		SmartDashboard.putNumber("CURRENT R", rightMotor.getStatorCurrent().getValueAsDouble());
	}

	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		//TODO: Clean up and make this actually work
		wheelSetpoint = velocity;
		var num = SmartDashboard.getNumber("Input/VOLTAGE", 0);
		leftMotor.setVoltage(num);
		rightMotor.set(num);
		SmartDashboard.putNumber("Input/VOLTAGE", num);
		leftMotor.setVoltage(wheelPID.calculate(leftMotor.getVelocity().getValue().in(RotationsPerSecond), wheelSetpoint.in(RotationsPerSecond)) + wheelFeedforward.calculate(wheelSetpoint.in(RotationsPerSecond)));
	}

	@Override
	public void setHoodAngle(Angle angle) {
		hoodSetpoint = angle;
		hoodMotor.set(hoodPID.calculate(hoodMotor.getPosition().getValue().div(HOOD_GEARING).in(Degrees), -hoodSetpoint.in(Degrees)));

		if (limitSwitch.get()) {
			hoodMotor.setPosition(0);
			hoodMotor.set(0);
		}
	}

	@Override
	public boolean atWheelVelocitySetpoint() {
		return (Math.abs(leftMotor.getVelocity().getValue().in(RotationsPerSecond) - wheelSetpoint.in(RotationsPerSecond)) < WHEEL_VELOCITY_TOLERANCE);
	}

	@Override
	public boolean atHoodAngleSetpoint() {
		return (Math.abs(hoodMotor.getPosition().getValue().div(HOOD_GEARING).in(Degrees) - hoodSetpoint.in(Degrees)) < HOOD_ANGLE_TOLERANCE_DEGREES);
	}
}
