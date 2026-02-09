package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.GlobalConstants.VOLTS;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.ShotSolver.ShooterMath;
import frc.robot.Subsystems.Shooter.ShotSolver.FuelSim.FuelSim;
import frc.robot.Subsystems.Shooter.ShotSolver.FuelSim.FuelSimSetup;

public class ShooterIOSim extends ShooterIOReal {

	private TalonFXSimState leftMotorSim;
	private TalonFXSimState rightMotorSim;
	private TalonFXSimState hoodMotorSim;
	private FlywheelSim wheelSim;
	private SingleJointedArmSim hoodSim;
	private boolean fuelLaunched = false;
    // TOF tracking
    private int lastHubScore = 0;
    private double lastShotStartStateTime = 0.0;

	public ShooterIOSim() {
		super();
		FuelSimSetup.setup();
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
			HOOD_MIN_ANGLE.in(Radians),
			HOOD_MAX_ANGLE.in(Radians),
			false, // this be ragebait
			HOOD_MIN_ANGLE.in(Radians)
		);

		// initialize lastHubScore to current score to avoid detecting old shots
		lastHubScore = FuelSim.Hub.BLUE_HUB.getScore();
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
		//if (atHoodAngleSetpoint() && atWheelVelocitySetpoint() && Shooter.getInstance().getState() != ShooterStates.IDLE) {
			if (GlobalConstants.Controllers.DRIVER_CONTROLLER.getPOV() != -1 && !fuelLaunched) {
				launchFuel();
				fuelLaunched = true;
			} else {
				if (GlobalConstants.Controllers.DRIVER_CONTROLLER.getPOV() == -1) {
					fuelLaunched = false;
				}
			}
			
		//}
		FuelSim.getInstance().updateSim();
		SmartDashboard.putNumber("SCORE", FuelSim.Hub.BLUE_HUB.getScore());

		// Check for new scored fuel and compute time-of-flight (TOF)
		int currentScore = FuelSim.Hub.BLUE_HUB.getScore();
		if (currentScore > lastHubScore) {
			double tof = Shooter.getInstance().getStateTime() - lastShotStartStateTime;
			SmartDashboard.putNumber("T_OF_F", tof);
			lastHubScore = currentScore;
		}
		ShooterMath.solveShot(Drive.getInstance().getPose(), new Translation2d(0,0), BLUE_HUB_POSE);
	}

	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		double wheelSpeed = SmartDashboard.getNumber("Wheel Setpoint (Rotations per Second)", 0);
		SmartDashboard.putNumber("Wheel Setpoint (Rotations per Second)", wheelSpeed);
		wheelSim.setInputVoltage(VOLTS * (wheelFeedforward.calculate(wheelSpeed * 2 * Math.PI) + wheelPID.calculate(wheelSim.getAngularVelocityRadPerSec(), wheelSpeed * 2 * Math.PI)));
	}

	@Override
	public void setHoodAngle(Angle angle) {
		double hoodGoal = SmartDashboard.getNumber("Hood Setpoint (Degrees)", 0);
		SmartDashboard.putNumber("Hood Setpoint (Degrees)", hoodGoal);
		double pid_calc = hoodPID.calculate(hoodSim.getAngleRads(), Units.degreesToRadians(hoodGoal));
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

	public void launchFuel() {
		double Jt = 2 * FLYWHEEL_MOI;
		double Vw = 0.0508 * wheelSim.getAngularVelocityRadPerSec();
		double rEff = 0.0508 / 2.0;
		double T = (20.0 * Jt)
                / (7.0 * 0.448 * 0.45392 * rEff * rEff + 40.0 * Jt);
		double Vp = Vw * T;
		double omegaFinal = wheelSim.getAngularVelocityRadPerSec() - (0.448 * 0.45392 * Vp * rEff) / Jt;
		wheelSim.setAngularVelocity(omegaFinal);
		double launchAngle = hoodSim.getAngleRads();
		// Make a Translation3d for velocity of the fuel using launch angle and Vp
		// Take into account robot orientation and that shooter is 90 deg CCW from robot forward
		// The horizontal (ground-plane) component is Vp * cos(launchAngle) in the shooter forward direction.
		// Shooter forward = robot forward rotated +90deg (CCW), so rotate by robot yaw + 90deg to get field-frame components.

		double robotYaw = Drive.getInstance().getPose().getRotation().getRadians();
		double shooterHeading = robotYaw + Math.toRadians(90.0); // 90 deg CCW from robot forward
		double horizontalSpeed = Vp * Math.cos(launchAngle);
		double dx = horizontalSpeed * Math.cos(shooterHeading);
		double dy = horizontalSpeed * Math.sin(shooterHeading);
		double dz = Vp * Math.sin(launchAngle);

		Translation3d launchVelocity = new Translation3d(dx, dy, dz);
		FuelSim.getInstance().spawnFuel(new Pose3d(Drive.getInstance().getPose()).getTranslation().plus(new Translation3d(0, 0.4, 0.3)), launchVelocity);
		// record the shooter state time at the moment of launch so we can compute TOF later
		lastShotStartStateTime = Shooter.getInstance().getStateTime();
	}
}