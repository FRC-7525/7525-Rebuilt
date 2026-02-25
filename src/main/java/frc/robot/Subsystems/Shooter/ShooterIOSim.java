package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.GlobalConstants.VOLTS;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.Subsystems.Drive.Drive;
// Removed unused import of ShooterMathNew; simulation currently calls launchFuel directly
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
			0, //
			2 * Math.PI, //
			false, // this be ragebait
			0
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
		if (Drive.getInstance().isAtSOTMTarget() && /*atHoodAngleSetpoint() && atWheelVelocitySetpoint()*/ Shooter.getInstance().getState() != ShooterStates.IDLE) {
			if (GlobalConstants.Controllers.DRIVER_CONTROLLER.getPOV() != -1 && !fuelLaunched) {
				launchFuel();
				fuelLaunched = true;
			} else {
				if (GlobalConstants.Controllers.DRIVER_CONTROLLER.getPOV() == -1) {
					fuelLaunched = false;
				}
			}
			
		}
		SmartDashboard.putBoolean("SHOOT_GOOD_DRIVE", Drive.getInstance().isAtSOTMTarget());
		FuelSim.getInstance().updateSim();
		SmartDashboard.putNumber("SCORE", FuelSim.Hub.BLUE_HUB.getScore());
		Logger.recordOutput(SUBSYSTEM_NAME + "/HMMMM", simulateShotTrajectory().stream().map((pose) -> pose).toArray(Pose3d[]::new));

		// Check for new scored fuel and compute time-of-flight (TOF)
		int currentScore = FuelSim.Hub.BLUE_HUB.getScore();
		if (currentScore > lastHubScore) {
			double tof = Shooter.getInstance().getStateTime() - lastShotStartStateTime;
			SmartDashboard.putNumber("T_OF_F", tof);
			lastHubScore = currentScore;
		}
		//ShooterMath.solveShot(Drive.getInstance().getPose(), Drive.getInstance().getVelocityTranslationFieldRelative(), BLUE_HUB_POSE);
	}

	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		wheelSetpoint = velocity;
		wheelSim.setInputVoltage(VOLTS * (wheelFeedforward.calculate(wheelSetpoint.in(RotationsPerSecond) * 2 * Math.PI) + wheelPID.calculate(wheelSim.getAngularVelocityRadPerSec(), wheelSetpoint.in(RotationsPerSecond) * 2 * Math.PI)));
	}

	@Override
	public void setHoodAngle(Angle angle) {
		hoodSetpoint = angle;
		double pid_calc = hoodPID.calculate(hoodSim.getAngleRads(), hoodSetpoint.in(Radians));
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
		// The horizontal (ground-plane) component is Vp * cos(launchAngle) in the shooter forward direction.
		// Compute shooter pose by applying ROBOT_TO_SHOOTER to the robot pose so we get correct position and yaw.
		Translation2d robotVelocity = Drive.getInstance().getVelocityTranslationFieldRelative();
		Pose3d robotPose3d = new Pose3d(Drive.getInstance().getPose());
		Pose3d shooterPose3d = robotPose3d.transformBy(ROBOT_TO_SHOOTER);
		double shooterHeading = shooterPose3d.getRotation().getZ();
		double horizontalSpeed = Vp * Math.cos(launchAngle);
		double dx = horizontalSpeed * Math.cos(shooterHeading) + robotVelocity.getX();
		double dy = horizontalSpeed * Math.sin(shooterHeading) + robotVelocity.getY();
		double dz = Vp * Math.sin(launchAngle);

		Translation3d launchVelocity = new Translation3d(dx, dy, dz);
		// Spawn at the shooter muzzle: transform a small local offset from the shooter frame into the field frame
		Pose3d muzzlePose = shooterPose3d.transformBy(new Transform3d(new Translation3d(0, 0.4, 0.3), new Rotation3d()));
		FuelSim.getInstance().spawnFuel(muzzlePose.getTranslation(), launchVelocity);
		// record the shooter state time at the moment of launch so we can compute TOF later
		lastShotStartStateTime = Shooter.getInstance().getStateTime();
	}

	/**
	 * Simulate the projectile trajectory for the current mechanism state.
	 * Returns a list of Pose3d samples representing the projectile position over time.
	 */
	public List<Pose3d> simulateShotTrajectory() {
		// Compute initial conditions same as launchFuel
		double Vw = 0.0508 * wheelSim.getAngularVelocityRadPerSec();
		double rEff = 0.0508 / 2.0;
		double Jt = 2 * FLYWHEEL_MOI;
		double T = (20.0 * Jt) / (7.0 * 0.448 * 0.45392 * rEff * rEff + 40.0 * Jt);
		double Vp = Vw * T; // projectile exit speed approx

		double launchAngle = hoodSim.getAngleRads();
		Translation2d robotVelocity = Drive.getInstance().getVelocityTranslationFieldRelative();
		double robotYaw = Drive.getInstance().getPose().getRotation().getRadians();
		Pose3d robotPose3d = new Pose3d(Drive.getInstance().getPose());
		Pose3d shooterPose3d = robotPose3d.transformBy(ROBOT_TO_SHOOTER);

		// include turret-tip tangential velocity from robot rotation
		double omega = Drive.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond;
		Translation3d offset3 = ShooterConstants.ROBOT_TO_SHOOTER.getTranslation();
		double rx = offset3.getX();
		double ry = offset3.getY();
		double tipX = -omega * ry;
		double tipY = omega * rx;
		// rotate tip velocity into field frame (rotate by robotYaw)
		double tipFieldX = tipX * Math.cos(robotYaw) - tipY * Math.sin(robotYaw);
		double tipFieldY = tipX * Math.sin(robotYaw) + tipY * Math.cos(robotYaw);

		double shooterHeading = shooterPose3d.getRotation().getZ();
		double horizontalSpeed = Vp * Math.cos(launchAngle);
		double vx = horizontalSpeed * Math.cos(shooterHeading) + robotVelocity.getX() + tipFieldX;
		double vy = horizontalSpeed * Math.sin(shooterHeading) + robotVelocity.getY() + tipFieldY;
		double vz = Vp * Math.sin(launchAngle);

		// Integrate trajectory with drag (use same physics constants as FuelSim)
		List<Pose3d> traj = new ArrayList<>();
		double t = 0.0;
		double dt = GlobalConstants.SIMULATION_PERIOD;
		// initial position (approx shooter exit) using ROBOT_TO_SHOOTER
		robotPose3d = new Pose3d(Drive.getInstance().getPose());
		shooterPose3d = robotPose3d.transformBy(ROBOT_TO_SHOOTER);
		Pose3d muzzlePose = shooterPose3d.transformBy(new Transform3d(new Translation3d(0, 0.4, 0.3), new Rotation3d()));
		Translation3d pos = muzzlePose.getTranslation();
		double px = pos.getX();
		double py = pos.getY();
		double pz = pos.getZ();

		// Constants copied from FuelSim for consistent drag behavior
		double airDensity = 1.2041; // kg/m^3
		double fuelRadius = 0.075; // m
		double fuelCrossArea = Math.PI * fuelRadius * fuelRadius; // m^2
		double dragCoef = 0.47; // dimensionless (smooth sphere)
		double dragForceFactor = 0.5 * airDensity * dragCoef * fuelCrossArea; // used as in FuelSim
		double fuelMass = 0.448 * 0.45392; // kg (same as FuelSim)

		// Current velocity vector
		double vx_i = vx;
		double vy_i = vy;
		double vz_i = vz;

		while (t < 5.0 && pz > -0.5) {
			traj.add(new Pose3d(new Translation3d(px, py, pz), new Rotation3d()));

			// Compute drag acceleration: Fd = -dragForceFactor * speed * v
			double speed = Math.sqrt(vx_i * vx_i + vy_i * vy_i + vz_i * vz_i);
			double ax = 0.0;
			double ay = 0.0;
			double az = -9.81; // gravity
			if (speed > 1e-6) {
				double dragAccFactor = -(dragForceFactor * speed) / fuelMass;
				ax += dragAccFactor * vx_i;
				ay += dragAccFactor * vy_i;
				az += dragAccFactor * vz_i; // note: vz component included in drag
			}

			// Integrate velocities
			vx_i += ax * dt;
			vy_i += ay * dt;
			vz_i += az * dt;

			// Integrate positions
			px += vx_i * dt;
			py += vy_i * dt;
			pz += vz_i * dt;

			t += dt;
		}

		return traj;
	}
}