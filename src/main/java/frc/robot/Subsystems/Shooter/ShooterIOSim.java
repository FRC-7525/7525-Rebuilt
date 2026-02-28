package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.GlobalConstants.VOLTS;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;

public class ShooterIOSim extends ShooterIOReal {

	private TalonFXSimState leftMotorSim;
	private TalonFXSimState rightMotorSim;
	private TalonFXSimState hoodMotorSim;
	private FlywheelSim wheelSim;
	private SingleJointedArmSim hoodSim;

	public ShooterIOSim() {
		super();
		hoodMotor = new TalonFX(HOOD_MOTOR_ID);
		rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned)); // Might need to be inverted
		rightMotorSim = new TalonFXSimState(rightMotor);
		leftMotorSim = new TalonFXSimState(leftMotor);
		hoodMotorSim = new TalonFXSimState(hoodMotor);

		wheelSim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(
				DCMotor.getKrakenX60(2),
				FLYWHEEL_MOI, // Moment of inertia
				FLYWHEEL_GEARING // Gearing
			),
			DCMotor.getKrakenX60(2)
		);
		hoodSim = new SingleJointedArmSim(
			LinearSystemId.createSingleJointedArmSystem(
				DCMotor.getFalcon500(1),
				HOOD_MOI, // Moment of inertia
				HOOD_GEARING // Gearing
			),
			DCMotor.getFalcon500(1),
			HOOD_GEARING,
			HOOD_ARM_LENGTH_METERS,
			HOOD_MIN_ANGLE.in(Radian),
			HOOD_MAX_ANGLE.in(Radian),
			false, // this be ragebait
			HOOD_MIN_ANGLE.in(Radian)
		);
	}

	@Override
	public void logOutputs(ShooterIOOutputs outputs) {
		// Sim update
		leftMotorSim.setRotorVelocity(Units.radiansToRotations(wheelSim.getAngularVelocityRadPerSec()));
		rightMotorSim.setRotorVelocity(Units.radiansToRotations(wheelSim.getAngularVelocityRadPerSec()));
		hoodMotorSim.setRawRotorPosition(Units.radiansToRotations(hoodSim.getAngleRads()));
		hoodMotorSim.setSupplyVoltage(VOLTS);
		hoodMotorSim.setRotorVelocity(Units.radiansToRotations(hoodSim.getVelocityRadPerSec()));
		wheelSim.update(GlobalConstants.SIMULATION_PERIOD);
		hoodSim.update(GlobalConstants.SIMULATION_PERIOD);

		outputs.leftWheelVelocity = leftMotor.getVelocity().getValue();
		outputs.rightWheelVelocity = rightMotor.getVelocity().getValue();
		outputs.wheelSetpoint = wheelSetpoint;
		outputs.hoodAngle = Radians.of(hoodSim.getAngleRads());
		outputs.hoodSetpoint = hoodSetpoint;
		// Throw some stuff here for 3D sim later
	}

	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		wheelSetpoint = velocity;
		SmartDashboard.putNumber("Wheel Setpoint (Radians per Second)", wheelSetpoint.in(RadiansPerSecond));
		wheelSim.setInputVoltage(VOLTS * (wheelPID.calculate(wheelSim.getAngularVelocityRadPerSec(), wheelSetpoint.in(RadiansPerSecond)) + wheelFeedforward.calculate(wheelSetpoint.in(RadiansPerSecond))));
	}

	@Override
	public void setHoodAngle(Angle angle) {
		hoodSetpoint = angle;
		double pid_calc = hoodPID.calculate(hoodSim.getAngleRads(), hoodSetpoint.in(Radians));
		SmartDashboard.putNumber("Hood PID Output", pid_calc);
		hoodSim.setInputVoltage(pid_calc * VOLTS);
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
