package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class ShooterIOReal implements ShooterIO {

	protected TalonFX leftMotor;
	protected TalonFX rightMotor;
	protected TalonFX hoodMotor;
	protected Angle hoodSetpoint;
	protected AngularVelocity wheelSetpoint;
	protected PIDController hoodPID;
	protected PIDController hoodDownPID;
	protected PIDController wheelPID;
	protected SimpleMotorFeedforward wheelFeedforward;
	protected TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
	protected TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
	protected DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
	protected VelocityVoltage wheelControlReq = new VelocityVoltage(0);

	public ShooterIOReal() {
		hoodPID = HOOD_PID.get();
		hoodPID.setSetpoint(0);
		hoodDownPID = HOOD_DOWN_PID.get();
		wheelPID = WHEEL_PID.get();
		wheelFeedforward = WHEEL_FEEDFORWARD.get();

		wheelSetpoint = RotationsPerSecond.zero();
		hoodSetpoint = Degrees.zero();
		leftMotor = new TalonFX(LEFT_SHOOTER_MOTOR_ID);
		leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		leftMotorConfig.Slot0.kP = wheelPID.getP();
		leftMotorConfig.Slot0.kI = wheelPID.getI();
		leftMotorConfig.Slot0.kD = wheelPID.getD();
		leftMotor.getConfigurator().apply(leftMotorConfig);

		rightMotor = new TalonFX(RIGHT_SHOOTER_MOTOR_ID);

		hoodMotor = new TalonFX(HOOD_MOTOR_ID);
		hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		hoodMotor.getConfigurator().apply(hoodMotorConfig);

		rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed)); // Might need to be inverted
	}

	@Override
	public void logOutputs(ShooterIOOutputs outputs) {
		outputs.leftWheelVelocity = leftMotor.getVelocity().getValue();
		outputs.rightWheelVelocity = rightMotor.getVelocity().getValue();
		outputs.wheelSetpoint = wheelSetpoint;
		outputs.hoodAngle = hoodMotor.getPosition().getValue().div(HOOD_GEARING);
		outputs.hoodSetpoint = hoodSetpoint;
		outputs.hoodCurrent = hoodMotor.getSupplyCurrent().getValueAsDouble();
		outputs.leftWheelCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
		outputs.rightWheelCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();

		SmartDashboard.putData("ShooterWheelPID", wheelPID);
		wheelFeedforward.setKv(SmartDashboard.getNumber("ShooterFeedforwardKv", wheelFeedforward.getKv()));
		SmartDashboard.putNumber("ShooterFeedforwardKv", wheelFeedforward.getKv());
		SmartDashboard.putData("ShooterHoodPID", hoodPID);
		SmartDashboard.putData("ShooterHoodDownPID", hoodDownPID);
	}

	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		wheelSetpoint = velocity;
		//TODO: Switch to this when done testing
		if (velocity.in(RotationsPerSecond) == 0) {
			leftMotor.stopMotor();
			return;
		}
		// leftMotor.setVoltage(wheelPID.calculate(leftMotor.getVelocity().getValue().in(RotationsPerSecond), wheelSetpoint.in(RotationsPerSecond)) + wheelFeedforward.calculate());
		//eftMotor.setVoltage(wheelPID.calculate(leftMotor.getVelocity().getValue().in(RotationsPerSecond), velocity.in(RotationsPerSecond)) + wheelFeedforward.calculate(wheelPID.getSetpoint());
		leftMotor.setControl(wheelControlReq.withVelocity(velocity).withFeedForward(wheelFeedforward.calculate(wheelSetpoint.in(RotationsPerSecond))));
	}

	@Override
	public void setHoodAngle(Angle angle) {
		hoodSetpoint = angle;
		hoodSetpoint = Degrees.of(SmartDashboard.getNumber("HOOD_ANGLE_TUNING", 0));
		SmartDashboard.putNumber("HOOD_ANGLE_TUNING", hoodSetpoint.in(Degrees));
		//TODO: Switch to this after done testing
		//TODO: Find a better way to do this lowkey cooked
		if (angle.in(Degrees) != 0) {
			hoodMotor.set(hoodPID.calculate(hoodMotor.getPosition().getValue().div(HOOD_GEARING).in(Degrees), hoodSetpoint.in(Degrees)));
		} else {
			if (!limitSwitch.get() && hoodMotor.getPosition().getValue().in(Degrees) != 0) {
				hoodMotor.set(0);
				hoodMotor.setPosition(0);
				hoodPID.reset();
				hoodDownPID.reset();
				return;
			}
			hoodMotor.set(hoodDownPID.calculate(hoodMotor.getPosition().getValue().div(HOOD_GEARING).in(Degrees), hoodSetpoint.in(Degrees)));
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

	@Override
	public boolean zeroHoodMotor() {
		hoodMotor.set(-.3);

		if (!limitSwitch.get()) {
			hoodMotor.setPosition(0);
			hoodMotor.set(0);
			return true;
		}
		return false;
	}

	public List<TalonFX> getShooterMotors() {
		return java.util.Arrays.asList(leftMotor, rightMotor);
	}

	public TalonFX getHoodMotor() {
		return hoodMotor;
	}
}
